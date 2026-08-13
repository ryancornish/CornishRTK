/* ============================================================================
 * Mutex contention under a timer storm
 *
 * This binary exists because of a hole in the suite rather than a hypothesis
 * about a bug. Every sync test in the tree builds `time_driver = "tickless"`,
 * so it has no tick at all, and every timer test builds without the `sync`
 * feature. The result is that the mutex, priority-inheritance and scheduler
 * machinery had never once run with a timer ISR firing underneath it. This is
 * the first build that combines the two.
 *
 * What that combination is worth exercising for, in the order it matters:
 *
 *   1. The port's signal masking. Interrupt-disable and preempt-disable are one
 *      signal mask on this port, and the model that keeps them straight is
 *      sensitive to how often signals actually arrive. A tick raises the
 *      delivery rate by orders of magnitude over anything else in the suite,
 *      which is the condition its assert has always been most likely to fire
 *      under. See preempt-port-masking-explained.md.
 *
 *   2. An ISR waking a HOLDER, which is the path cross-core-defects.md 8.2
 *      argues is sound and nothing executes. wake_one is documented ISR-safe,
 *      and it reaches set_thread_ready, which folds the woken thread's urgency
 *      and can therefore take remote queue spinlocks with local interrupts
 *      disabled, recursing to the inheritance depth bound. Core 3's worker
 *      blocks on a semaphore WHILE HOLDING a contended mutex, and its own
 *      core's timer ISR is what releases it, so every tick on core 3 drives
 *      that fold from interrupt context.
 *
 *   3. Bring-up and shutdown. Ticks arriving across the dormant-region
 *      transitions, which are depth changes on the same mask, and which a
 *      single long-running test would never sample. Hence several lifecycles.
 *
 * WHAT THIS DOES NOT YET COVER, and do not assume otherwise. The interleaving
 * CLAUDE_cyros.md section 7 actually asks for is a tick landing INSIDE a
 * reschedule, and that is unreachable in the tree as it stands: stage 2 is not
 * applied, so install_interceptor still passes timer_signo in block_extra and
 * the timer is masked for the whole interception. This binary is the
 * prerequisite fuzzer for that change rather than evidence about it. Once stage
 * 2 lands, this test becomes that fuzzer with no modification, and the honest
 * order of work is to land stage 2 and then soak this.
 *
 * The assertions here are deliberately weak, because the value is in the soak
 * rather than in any one run. A run that neither trips an assert nor hangs is
 * the result, and the checks below exist mostly to stop a run that did nothing
 * from looking like a run that survived something. The tick counters are the
 * vacuity guard: if the storm did not fire, the test failed to test anything
 * and says so rather than passing quietly.
 *
 * Lock order is m_outer then m_inner everywhere it is nested, so nothing here
 * can deadlock on its own. Each core carries exactly one thread, so the port's
 * no-slice-timer starvation rule (a same-core non-yielding spinner deadlocks)
 * cannot bite either.
 * ========================================================================= */

#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;
static_assert(config::cores >= 4, "Timer storm is designed for a quad-core configuration");

namespace
{

constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* 10 kHz, with the storm re-arming every single tick. That is the highest rate
 * the driver can be asked for, and the point is to maximise how often a signal
 * lands inside a critical section rather than to model any real workload. */
constexpr std::uint32_t tick_hz               = 10'000;
constexpr std::uint64_t storm_interval_ticks  = 1;

/* Iterations the pacing worker runs before winding everything down, and how
 * many kernel lifecycles to do it in. Sized so one run is under two seconds and
 * lands tens of thousands of ticks inside lock and unlock paths.
 *
 * Several lifecycles rather than one because bring-up and shutdown are their
 * own stress here: they are the dormant-region transitions, and a tick arriving
 * across one of those is an interleaving a single long run never samples. */
constexpr std::uint64_t pace_iterations = 20'000;
constexpr int           reps            = 3;

struct alignas(CYROS_PORT_STACK_ALIGN) aligned_stack
{
   std::array<std::byte, STACK_SIZE> bytes;
};

struct storm_state
{
   mutex m_outer;                  // held across a semaphore block by core 3
   mutex m_inner;                  // the contended one
   sync::semaphore isr_gate{0};    // released from timer ISR context on core 3

   std::array<std::atomic<std::uint64_t>, 4> ticks{};
   std::array<std::atomic<std::uint64_t>, 4> iterations{};
   std::atomic<std::uint64_t> isr_wakes{0};
   std::atomic<bool> stop{false};
};



/// Ordinary storm tick. Does nothing but prove the core's timer is live.
void tick_callback(void* arg) noexcept
{
   auto* s = static_cast<storm_state*>(arg);
   s->ticks[cyros_port_get_core_id()].fetch_add(1, std::memory_order_relaxed);
}


/// The storm tick on core 3, which additionally wakes a thread that is holding
/// a contended mutex. That is the 8.2 path: wake_one from interrupt context,
/// into set_thread_ready, into the urgency fold, which may take the queue lock
/// of a mutex other cores are actively contending.
void isr_wake_callback(void* arg) noexcept
{
   auto* s = static_cast<storm_state*>(arg);
   s->ticks[cyros_port_get_core_id()].fetch_add(1, std::memory_order_relaxed);
   s->isr_wakes.fetch_add(1, std::memory_order_relaxed);
   s->isr_gate.release();
}

/// Arm this core's own timer. start() is per-core and must run in the core's
/// own context, which is why every worker does this for itself rather than the
/// test body doing it once.
time::handle arm_storm(storm_state& s, time::callback cb)
{
   auto h = time::schedule_recurring(time::duration{storm_interval_ticks}, cb, &s);
   time::start();
   return h;
}

void disarm_storm(time::handle h)
{
   (void)time::cancel(h);
   time::stop();
}

class MutexTimerStorm_Test : public ::testing::Test {};

}  // namespace


TEST_F(MutexTimerStorm_Test, GivenAHighRateTickOnEveryCore_WhenMutexesAreContended_ThenTheKernelSurvives)
{
   for (int rep = 0; rep < reps; ++rep) {
   SCOPED_TRACE("rep " + std::to_string(rep));

   time::initialise(tick_hz);   // global, and must precede kernel::start()
   kernel::initialise();

   static std::array<aligned_stack, 4> stacks{};
   storm_state s;

   // core0: the pacer. Contends for the inner mutex and decides when to stop.
   thread w0(
      [&s]{
         auto h = arm_storm(s, tick_callback);
         for (std::uint64_t i = 0; i < pace_iterations; ++i) {
            s.m_inner.lock();
            s.m_inner.unlock();
            s.iterations[0].fetch_add(1, std::memory_order_relaxed);
         }
         s.stop.store(true, std::memory_order_release);

         disarm_storm(h);
      },
      stacks[0].bytes, thread::priority(5), core0);

   // core1: nests inner inside outer, so it queues behind core3's holder while
   // itself holding nothing, and behind core0/core2 on the inner one. This is
   // what puts bridges on the queues the ISR fold has to walk.
   thread w1(
      [&s]{
         auto h = arm_storm(s, tick_callback);
         while (!s.stop.load(std::memory_order_acquire)) {
            s.m_outer.lock();
            s.m_inner.lock();
            s.m_inner.unlock();
            s.m_outer.unlock();
            s.iterations[1].fetch_add(1, std::memory_order_relaxed);
         }
         disarm_storm(h);
      },
      stacks[1].bytes, thread::priority(4), core1);

   // core2: pure contention on the inner mutex, at the best base priority, so
   // it is the thread most likely to be the one a boost has to beat.
   thread w2(
      [&s]{
         auto h = arm_storm(s, tick_callback);
         while (!s.stop.load(std::memory_order_acquire)) {
            s.m_inner.lock();
            s.m_inner.unlock();
            s.iterations[2].fetch_add(1, std::memory_order_relaxed);
         }
         disarm_storm(h);
      },
      stacks[2].bytes, thread::priority(3), core2);

   // core3: holds the outer mutex across a block on the semaphore, and is woken
   // by its OWN core's timer ISR. Worst base priority, so while it holds the
   // outer mutex it genuinely needs a boost to run again.
   thread w3(
      [&s]{
         auto h = arm_storm(s, isr_wake_callback);
         while (!s.stop.load(std::memory_order_acquire)) {
            s.m_outer.lock();
            s.isr_gate.acquire();     // parks HOLDING m_outer, woken from an ISR
            s.m_outer.unlock();
            s.iterations[3].fetch_add(1, std::memory_order_relaxed);
         }
         disarm_storm(h);
      },
      stacks[3].bytes, thread::priority(6), core3);

   kernel::start();
   kernel::finalise();
   time::finalise();

   std::uint64_t total_ticks = 0;
   for (std::size_t c = 0; c < 4; ++c) {
      total_ticks += s.ticks[c].load();
   }

   // Vacuity guards first: a run where the storm never fired, or where a core's
   // timer never started, proves nothing about surviving one.
   EXPECT_GT(total_ticks, 1'000u)
      << "the timer storm barely fired (" << total_ticks
      << " ticks), so this run did not exercise what it claims to";
   for (std::size_t c = 0; c < 4; ++c) {
      EXPECT_GT(s.ticks[c].load(), 0u)
         << "core " << c << " never took a timer interrupt, so its own timer "
            "never armed and that core ran unstormed";
   }
   EXPECT_GT(s.isr_wakes.load(), 0u)
      << "the ISR never woke the holder, so the 8.2 path was not exercised";

   // Progress: every worker has to have got somewhere, or the storm starved the
   // system rather than stressing it.
   for (std::size_t c = 0; c < 4; ++c) {
      EXPECT_GT(s.iterations[c].load(), 0u)
         << "worker on core " << c << " completed no iterations under the storm";
   }
   }
}
