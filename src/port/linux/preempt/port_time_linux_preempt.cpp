/**
 * @file port_time_linux_preempt.cpp
 * @brief Linux timer-driver port for the preemptive (sigctx) backend.
 *
 * Unlike the coop time port, which is a passive counter that tests pump by
 * hand, this one is a real asynchronous interrupt source. Each core owns a POSIX
 * timer that fires a dedicated signal at that core's own thread, and the signal
 * handler invokes the ISR the time driver registered. That is what lets a timer
 * preempt a running thread the way real hardware does.
 *
 * Per-core model
 * --------------
 * Every core has its own timer, created by that core when it calls time::start().
 * A core's timer is delivered via SIGEV_THREAD_ID to that core's own kernel TID,
 * so the ISR runs on the core the timer belongs to. cyros_port_get_core_id()
 * inside the ISR therefore returns that core, and the driver services that core's
 * own domain. Per-core delivery and the driver's per-core indexing compose with
 * no core id threaded through the signal: it falls out of which thread the signal
 * lands on.
 *
 * The clock is shared. now() is one CLOCK_MONOTONIC source every core reads. Only
 * the timer hardware (comparator deadline, tick rate) is per-core, which mirrors
 * a per-core comparator over a shared monotonic counter.
 *
 * Signal model
 * ------------
 * The timer uses its OWN signal (timer_signo), distinct from the reschedule
 * signal the main port uses. That separation is what makes the interrupt-versus-
 * preempt distinction real: the main port masks timer_signo under interrupt-
 * disable but leaves it unmasked under preempt-disable, so a timer interrupt
 * still fires inside a preempt-disabled region and only the reschedule it
 * requests is deferred.
 *
 * The timer handler runs with the reschedule signal masked (set in sa_mask
 * below), so a reschedule cannot nest into the middle of the ISR. Any wake the
 * ISR performs pends a reschedule that the kernel delivers when the handler
 * returns. The process-wide sigaction is installed once, the first time any core
 * calls setup(); the timers themselves are per-core.
 *
 * The reverse direction is closed symmetrically by the main port. Its reschedule
 * interceptor is installed with sigctx's block_extra set to timer_signo, so the
 * timer stays masked for the whole interception, capture and handler alike. A
 * timer can therefore never land on the shared handler stack mid-reschedule, and
 * the two signals can never nest into each other in either order.
 *
 * now() is backed by CLOCK_MONOTONIC, so time advances on its own. The
 * deterministic cyros_port_time_advance() hook the periodic and tickless driver
 * tests rely on is therefore unsupported here. Those suites stay on the coop
 * port.
 */

#include <cyros/port/port.h>
#include <cyros/port/port_time.h>

#include "port_linux_preempt_internal.hpp"

#include <array>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <mutex>
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>

namespace
{

/**
 * Must match timer_signo in port_linux_preempt.cpp. If these drift, interrupt-
 * disable will not actually hold off the timer ISR. SIGRTMIN is not a constant
 * expression, so this is a runtime-initialised const.
 */
const int timer_signo = SIGRTMIN;

/**
 * Must match preempt_signo in port_linux_preempt.cpp. The timer handler masks it
 * so a reschedule cannot nest into the ISR.
 */
const int reschedule_signo = SIGURG;

/** 1 tick == 1 microsecond, matching the coop port's convention. */
constexpr uint64_t tick_freq_hz = 1'000'000;

/**
 * @brief One core's timer hardware.
 *
 * Created by the owning core in setup() and armed/disarmed only by that core, so
 * the fields need no cross-core synchronisation beyond the atomic deadline the
 * ISR and thread context both touch.
 */
struct core_timer
{
   std::atomic<uint64_t> armed_deadline{UINT64_MAX};
   timer_t  timer{};
   bool     timer_created{false};
   uint32_t tick_hz{0};    // 0 means tickless one-shot
   pid_t    tid{0};        // owning core's kernel TID, target of its timer signal
};

struct time_state
{
   // Shared: one handler (the driver's trampoline) for every core's timer.
   std::atomic<cyros_port_isr_handler_t> isr{nullptr};
   std::atomic<void*>                    isr_arg{nullptr};

   // Shared clock. now() == base_ticks + (CLOCK_MONOTONIC now - epoch) in ticks.
   std::atomic<uint64_t> base_ticks{0};
   std::atomic<uint64_t> epoch_ns{0};

   // Per-core timer hardware.
   std::array<core_timer, CYROS_PORT_CORE_COUNT> core{};
};
time_state ts;

uint64_t monotonic_ns()
{
   struct timespec t;
   clock_gettime(CLOCK_MONOTONIC, &t);
   return static_cast<uint64_t>(t.tv_sec) * 1'000'000'000ull + static_cast<uint64_t>(t.tv_nsec);
}

uint64_t ns_to_ticks(uint64_t ns)
{
   return ns / (1'000'000'000ull / tick_freq_hz);
}

uint64_t ticks_to_ns(uint64_t ticks)
{
   return ticks * (1'000'000'000ull / tick_freq_hz);
}

/// @brief The calling core's timer hardware.
core_timer& this_core_timer()
{
   return ts.core[cyros_port_get_core_id()];
}

void program_oneshot_ns(core_timer& ct, uint64_t rel_ns)
{
   if (!ct.timer_created) return;
   if (rel_ns == 0) rel_ns = 1; // 0 would disarm, so fire as soon as possible instead

   struct itimerspec its;
   its.it_value.tv_sec     = static_cast<time_t>(rel_ns / 1'000'000'000ull);
   its.it_value.tv_nsec    = static_cast<long>(rel_ns % 1'000'000'000ull);
   its.it_interval.tv_sec  = 0;
   its.it_interval.tv_nsec = 0;
   timer_settime(ct.timer, 0, &its, nullptr);
}

void interceptor_disarm(core_timer& ct)
{
   if (!ct.timer_created) return;
   struct itimerspec its;
   its.it_value.tv_sec     = 0;
   its.it_value.tv_nsec    = 0;
   its.it_interval.tv_sec  = 0;
   its.it_interval.tv_nsec = 0;
   timer_settime(ct.timer, 0, &its, nullptr);
}

void on_timer_signal(int, siginfo_t*, void*)
{
   // Runs on the core whose timer fired (SIGEV_THREAD_ID targeted this thread),
   // with the reschedule signal masked so a reschedule cannot nest in here.
   // Invoke the driver's ISR, which reads cyros_port_get_core_id() and so
   // services this core's domain. Any reschedule that readying requests is
   // pended and delivered once this handler returns.

   // Delivery blocked timer_signo and, via sa_mask, reschedule_signo, which is
   // exactly the mask interrupt-disable calls for. Declaring the region keeps a
   // cyros_port_irq_restore() inside the ISR from reopening both signals
   // mid-handler and letting a timer nest on the altstack.
   cyros::port::isr_region const in_isr;

   auto handler = ts.isr.load(std::memory_order_acquire);
   if (handler) {
      handler(ts.isr_arg.load(std::memory_order_acquire));
   }
}

/// @brief Install the process-wide timer signal handler exactly once.
void ensure_signal_handler_installed()
{
   static std::once_flag once;
   std::call_once(once, [] {
      // The timer ISR runs with the reschedule signal masked so a switch cannot
      // nest into it, on the altstack to keep off the interrupted thread's stack.
      struct sigaction sa;
      memset(&sa, 0, sizeof(sa));
      sa.sa_sigaction = on_timer_signal;
      sigemptyset(&sa.sa_mask);
      sigaddset(&sa.sa_mask, reschedule_signo);
      sa.sa_flags = SA_SIGINFO | SA_RESTART | SA_ONSTACK;
      sigaction(timer_signo, &sa, nullptr);
   });
}

} // namespace


/* ============================================================================
 * Time Driver Port Contract API
 * ========================================================================= */

void cyros_port_time_setup(uint32_t tick_hz)
{
   // Called by the calling core during its time::start(). Installs the shared
   // signal handler once, then creates THIS core's timer targeting THIS core's
   // thread, so its ISR runs here and services this core's domain.
   ensure_signal_handler_installed();

   core_timer& ct = this_core_timer();

   // Idempotent: a repeated setup (e.g. a fresh test's start() after a prior
   // finalise()) deletes the previous timer before creating a new one, so timers
   // do not leak across init/finalise cycles.
   if (ct.timer_created) {
      timer_delete(ct.timer);
      ct.timer_created = false;
   }

   ct.tick_hz = tick_hz;
   ct.tid     = static_cast<pid_t>(syscall(SYS_gettid));

   struct sigevent sev;
   memset(&sev, 0, sizeof(sev));
   sev.sigev_notify = SIGEV_THREAD_ID;
   sev.sigev_signo  = timer_signo;
   // sigev_notify_thread_id is a glibc macro for _sigev_un._tid, gated on
   // _GNU_SOURCE having been visible before signal.h's first inclusion anywhere
   // in the translation unit. That's a transitive, include-order-dependent
   // condition this file cannot guarantee given upstream port/config headers, so
   // write the union member directly instead of relying on the macro existing.
   sev._sigev_un._tid = ct.tid;

   if (timer_create(CLOCK_MONOTONIC, &sev, &ct.timer) == 0) {
      ct.timer_created = true;
   }
}

uint64_t cyros_port_time_now(void)
{
   uint64_t base  = ts.base_ticks.load(std::memory_order_acquire);
   uint64_t epoch = ts.epoch_ns.load(std::memory_order_acquire);
   uint64_t cur   = monotonic_ns();
   uint64_t delta = (cur > epoch) ? (cur - epoch) : 0;
   return base + ns_to_ticks(delta);
}

uint64_t cyros_port_time_freq_hz(void)
{
   return tick_freq_hz;
}

void cyros_port_time_reset(uint64_t t)
{
   // Resets the shared clock, and clears every core's armed deadline. Intended
   // for deterministic startup in tests.
   ts.epoch_ns.store(monotonic_ns(), std::memory_order_release);
   ts.base_ticks.store(t, std::memory_order_release);

   for (auto& ct : ts.core) {
      ct.armed_deadline.store(UINT64_MAX, std::memory_order_release);
   }
}

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t h, void* arg)
{
   ts.isr_arg.store(arg, std::memory_order_relaxed);
   ts.isr.store(h, std::memory_order_release);
}

void cyros_port_time_irq_enable(void)
{
   core_timer& ct = this_core_timer();
   if (!ct.timer_created) return;

   if (ct.tick_hz > 0) {
      // Periodic tick on this core's timer.
      uint64_t period_ns = 1'000'000'000ull / ct.tick_hz;
      struct itimerspec its;
      its.it_value.tv_sec     = static_cast<time_t>(period_ns / 1'000'000'000ull);
      its.it_value.tv_nsec    = static_cast<long>(period_ns % 1'000'000'000ull);
      its.it_interval         = its.it_value;
      timer_settime(ct.timer, 0, &its, nullptr);
   } else {
      // Tickless: nothing to start until a deadline is armed on this core.
      uint64_t deadline = ct.armed_deadline.load(std::memory_order_acquire);
      if (deadline != UINT64_MAX) {
         uint64_t now = cyros_port_time_now();
         uint64_t rel = (deadline > now) ? (deadline - now) : 0;
         program_oneshot_ns(ct, ticks_to_ns(rel));
      }
   }
}

void cyros_port_time_irq_disable(void)
{
   interceptor_disarm(this_core_timer());
}

void cyros_port_time_arm(uint64_t deadline)
{
   core_timer& ct = this_core_timer();

   // Keep the earliest deadline for this core.
   uint64_t cur = ct.armed_deadline.load(std::memory_order_relaxed);
   while (deadline < cur &&
          !ct.armed_deadline.compare_exchange_weak(cur, deadline,
                                                   std::memory_order_release,
                                                   std::memory_order_relaxed))
   {}

   uint64_t now = cyros_port_time_now();
   uint64_t rel = (deadline > now) ? (deadline - now) : 0;
   program_oneshot_ns(ct, ticks_to_ns(rel));
}

void cyros_port_time_disarm(void)
{
   core_timer& ct = this_core_timer();
   ct.armed_deadline.store(UINT64_MAX, std::memory_order_release);
   interceptor_disarm(ct);
}

// Linux-only test hook. The preempt port is backed by a real monotonic clock,
// so deterministic advance is unsupported. Pumped, deterministic time-driver
// tests should use the coop port.
extern void cyros_port_time_advance(uint64_t delta)
{
   (void)delta;
}

void cyros_port_send_time_ipi(uint32_t core_id)
{
   // Nudge the given core to do time work by raising its timer signal. Its ISR
   // runs on that core and services that core's domain.
   pid_t tid = ts.core[core_id].tid;
   if (tid != 0) {
      syscall(SYS_tgkill, getpid(), tid, timer_signo);
   }
}
