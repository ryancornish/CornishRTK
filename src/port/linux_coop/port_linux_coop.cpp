/**
 * @file port_linux_coop.cpp
 * @brief Linux simulation port, cooperative switching
 *
 * It simulates embedded behavior (stack-based context switching) while
 * running on Linux for development and testing.
 *
 * SMP support: Each pthread represents a "core". Use cyros_port_get_core_id()
 * to determine which simulated core is running.
 *
 * Reschedule model
 * ----------------
 * This port implements the two-operation reschedule contract from port.h:
 *
 *   cyros_port_thread_yield()    - strong guarantee, synchronous. Resumes the
 *                                   scheduler context immediately. Caller must be
 *                                   in thread context at baseline priority.
 *
 *   cyros_port_pend_reschedule() - weak guarantee, deferred-safe. If invoked
 *                                   at baseline priority it resolves now (the
 *                                   same synchronous switch). If invoked while the core is
 *                                   kernel-masked it sets a per-core
 *                                   "reschedule pending" flag instead.
 *
 * Baseline priority and the two depth counters
 * --------------------------------------------
 * "Kernel-masked" on this port means EITHER counter is non-zero:
 *   - interrupt_disable_depth : interrupt masking (Critical Sections).
 *   - preempt_disable_depth   : preemption disabling (Preemption Control).
 * A context switch may occur only at "baseline priority": inside a thread
 * context with BOTH counters at zero.
 *
 * Resolving the pending flag
 * --------------------------
 * The flag is drained at whichever safe point is reached last - that is,
 * whenever a counter returns to zero and the OTHER counter is already zero:
 *   - cyros_port_irq_restore()  reaching interrupt depth 0, and
 *   - cyros_port_preempt_enable() reaching preempt depth 0.
 * Both paths funnel through resolve_pending_reschedule_if_baseline(), which
 * only acts when the full baseline condition holds.
 */

#include <cyros/port/port.h>

/* Deliberately use the 'private' header details rather than <boost/context/fiber.hpp>.
 * Using the public fiber interface requires compiling with exceptions (for a feature
 * we don't need). Though this is 'detail' it *shouldn't* change.
 */
#include <boost/context/detail/fcontext.hpp>
namespace boost::context
{

using detail::fcontext_t;
using detail::transfer_t;
using detail::make_fcontext;
using detail::jump_fcontext;

}

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>


/* ============================================================================
 * Context handles and the transfer protocol
 * ========================================================================= */

/**
 * @brief Handle to a suspended CPU context.
 *
 * Wrapper around raw boost::context::fcontext_t that implements move semantics.
 */
class context_handle
{
public:
   constexpr context_handle() = default;
   constexpr explicit context_handle(boost::context::fcontext_t ctx) noexcept : fctx(ctx) {}
   ~context_handle() = default;

   context_handle(context_handle const&)            = delete;
   context_handle& operator=(context_handle const&) = delete;

   context_handle(context_handle&& other) noexcept : fctx(std::exchange(other.fctx, nullptr)) {}
   context_handle& operator=(context_handle&& other) noexcept
   {
      fctx = std::exchange(other.fctx, nullptr);
      return *this;
   }

   /**
    * @brief True while this handle refers to a context that can be resumed.
    */
   explicit operator bool() const noexcept { return fctx != nullptr; }

   /**
    * @brief Switch to this context, consuming the handle.
    * @param data Handed to the target as transfer_t::data, see the protocol below.
    * @return The context control came back from, and whatever data it sent.
    */
   boost::context::transfer_t jump(void* data)
   {
      CYROS_ASSERT(fctx); // Bug: switching to an empty or already-spent context
      return boost::context::jump_fcontext(std::exchange(fctx, nullptr), data);
   }

private:
   boost::context::fcontext_t fctx{nullptr};
};

/**
 * @brief transfer_t::data protocol.
 *
 * A make_fcontext entry point receives its argument through the data pointer of
 * the FIRST jump into it, and a finishing thread has to announce itself
 * explicitly. boost::context signalled completion by returning an empty fiber,
 * use a 'thread_finished_signal' as a sentinel for the same behaviour.
 *
 *   first entry into a thread      : the cyros_port_context*
 *   first entry into a scheduler   : the cpu_core*
 *   ordinary suspend, either way   : nullptr
 *   a thread's final jump out      : thread_finished_signal
 */
static std::byte   thread_finished_tag;  // Address-only sentinel, never read
static void* const thread_finished_signal = &thread_finished_tag;

/* ============================================================================
 * Port Context Structure
 * ========================================================================= */

struct cyros_port_context
{
   context_handle       thread;  // Thread context (owned by scheduler when idle)
   void*                stack_top;
   size_t               stack_size;
   cyros_port_entry_t   entry;
   void*                arg;
};

/* ============================================================================
 * Verify Port Traits
 * ========================================================================= */
static_assert(sizeof(cyros_port_context) == CYROS_PORT_CONTEXT_SIZE,
              "CYROS_PORT_CONTEXT_SIZE mismatch - adjust in port_traits.h");
static_assert(alignof(cyros_port_context) == CYROS_PORT_CONTEXT_ALIGN,
              "CYROS_PORT_CONTEXT_ALIGN mismatch - adjust in port_traits.h");
static_assert((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) == 0,
              "CYROS_PORT_STACK_ALIGN must be a power of two");



/**
 * For simulating the SMP schedulers on top of linux, we
 * spawn a new pthread (AKA OS-thread) for each configured core - (Except core0 because that is the initial thread).
 * Each OS-thread encapsulates a scheduler context that is their "outer-context" for each user-thread pinned to a core.
 * When the user-thread "pends reschedule", the context is switched to this outer context so that scheduling can happen
 * on a separate stack and context to the threads.
 */
struct cpu_core
{
   pthread_t pthread{}; // @note This is null/unused for core0
   uint32_t  core_id{}; // Index from 0... num cores
   cyros_port_core_entry_t entry{};
   context_handle scheduler_context;

   /**
    * @brief The scheduler context's stack, owned by this core.
    *
    * boost::context::fiber allocated this itself, with a guard page. Raw
    * fcontext has no allocator, so the port owns it - and keeps the guard,
    * because a scheduler-stack overflow would otherwise scribble silently over
    * the neighbouring cpu_core instead of faulting somewhere findable.
    *
    * Layout: [ guard page (PROT_NONE) ][ stack, grows down into the guard ]
    */
   static constexpr size_t scheduler_stack_size = 128 * 1024; // Probably size overkill
   void*  scheduler_mapping{nullptr};
   size_t scheduler_mapping_size{0};

   ~cpu_core();

   /**
    * @brief Primitives to signal/communicate with other cores
    */
   struct core_poke
   {
      pthread_mutex_t mutex{};
      pthread_cond_t  cond_var{};
      std::atomic<bool> pending{false}; // Can be set by any core
      core_poke()  { pthread_mutex_init(&mutex, nullptr); pthread_cond_init(&cond_var, nullptr); }
      ~core_poke() { pthread_cond_destroy(&cond_var); pthread_mutex_destroy(&mutex); }
   } core_poke;

   void start_scheduler();
};

/**
 * @brief Dynamically-sized array/container wrapper for cpu_core's
 *
 * Using a std::vector _would_ be preferable, but impossible as a cpu_core is non-copyable
 */
class cpu_core_array
{
public:
   using iterator = cpu_core*;
   using const_iterator = const cpu_core*;

   constexpr cpu_core_array() = default;
   explicit cpu_core_array(size_t count, cyros_port_core_entry_t core_entry)
      : cores(std::make_unique<cpu_core[]>(count)), count(count)
   {
      for (uint32_t i = 0; i < count; ++i) {
         cores[i].core_id = i;
         cores[i].entry = core_entry;
      }
   }
   cpu_core_array(cpu_core_array&&) noexcept            = default;
   cpu_core_array& operator=(cpu_core_array&&) noexcept = default;
   cpu_core_array(cpu_core_array const&)            = delete;
   cpu_core_array& operator=(cpu_core_array const&) = delete;

   cpu_core&       operator[](size_t index)       { return cores[index]; }
   cpu_core const& operator[](size_t index) const { return cores[index]; }

   [[nodiscard]] size_t size() const { return count; }

   iterator begin() { return cores.get(); }
   iterator end()   { return cores.get() + count; }

   [[nodiscard]] const_iterator begin() const { return cores.get(); }
   [[nodiscard]] const_iterator end()   const { return cores.get() + count; }

private:
   std::unique_ptr<cpu_core[]> cores;
   size_t count{0};
};


struct global_state
{
   std::atomic<bool>        shutdown_requested{false};
   std::atomic<uint32_t>    active_contexts{0};
   cyros_port_reschedule_t reschedule_handler{nullptr};
   cpu_core_array cores;

   /// @brief Have any cores been started?
   [[nodiscard]] bool cores_launched() const { return cores.size() > 0; }

   void reset()
   {
      shutdown_requested.store(false);
      active_contexts.store(0);
      reschedule_handler = nullptr;
      cores = {};
   }
};
static constinit global_state global;

/**
 * @brief Per-OS-Thread state
 *
 * For SMP simulation, each OS-thread has its own state tracking using
 * thread_local.
 */
struct current_core_state
{
   cpu_core* core{nullptr};

   // Non-null when we are currently executing inside a thread context.
   cyros_port_context* current_context{nullptr};

   // The "caller" context for the currently running thread on *this OS-thread*.
   context_handle thread_caller;
   // The outermost context that takes us back out of the scheduler when it is finished.
   context_handle os_caller;
   // Simulates pointing to a context's dedicated TLS block.
   void* tls_pointer{nullptr};

   // Interrupt-masking nesting depth (Critical Sections). Blocks the hardware.
   uint32_t interrupt_disable_depth{0};

   // Preemption-disable nesting depth (Preemption Control). Blocks the
   // scheduler from switching while non-zero; interrupts still flow.
   uint32_t preempt_disable_depth{0};

   // Set by cyros_port_pend_reschedule() when it cannot resolve immediately
   // (i.e. the core is kernel-masked). Drained at the next safe point: either
   // irq_restore() reaching interrupt depth 0, or preempt_enable() reaching
   // preempt depth 0 - whichever leaves BOTH counters at zero.
   bool reschedule_pending{false};
};
static thread_local constinit current_core_state current_core;

static size_t guard_page_size()
{
   auto const page = sysconf(_SC_PAGESIZE);
   return page > 0 ? static_cast<size_t>(page) : 4096u;
}

cpu_core::~cpu_core()
{
   if (scheduler_mapping != nullptr) {
      munmap(scheduler_mapping, scheduler_mapping_size);
   }
}

/**
 * @brief Entry point of a core's scheduler context.
 *
 * @note Must never return. A make_fcontext entry that returns is undefined
 *       behaviour
 */
[[gnu::noreturn]]
static void scheduler_trampoline(boost::context::transfer_t entry_transfer)
{
   auto* core = static_cast<cpu_core*>(entry_transfer.data);
   current_core.os_caller = context_handle(entry_transfer.fctx);

   // Run kernel entry for this simulated core (will start first thread etc.)
   core->entry();

   // Cooperative pump until only idle threads remain (one idle thread per core).
   while (global.active_contexts.load(std::memory_order_acquire) > global.cores.size()) {
      global.reschedule_handler();
   }

   // First core to observe quiescence initiates shutdown.
   bool expected = false;
   if (global.shutdown_requested.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      for (auto& c : global.cores) {
         cyros_port_send_reschedule_ipi(c.core_id);
      }
   }

   current_core.os_caller.jump(nullptr);

   CYROS_PORT_UNREACHABLE(); // the jump above does not come back
}

void cpu_core::start_scheduler()
{
   size_t const guard = guard_page_size();
   scheduler_mapping_size = guard + scheduler_stack_size;

   scheduler_mapping = mmap(nullptr, scheduler_mapping_size,
                            PROT_READ | PROT_WRITE,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
   CYROS_ASSERT(scheduler_mapping != MAP_FAILED); // Out of memory for a scheduler stack

   // Lowest page is the guard: the stack grows down towards it.
   int const guarded = mprotect(scheduler_mapping, guard, PROT_NONE);
   CYROS_ASSERT(guarded == 0); // Could not arm the scheduler stack guard page
   (void)guarded;

   auto* const stack_top = static_cast<uint8_t*>(scheduler_mapping) + scheduler_mapping_size;

   scheduler_context = context_handle(
      boost::context::make_fcontext(stack_top, scheduler_stack_size, &scheduler_trampoline)
   );

   // Enter the scheduler context. It jumps back here when this core is done,
   // and jump() has already emptied the handle by then.
   scheduler_context.jump(this);
}

/**
 * @brief Perform the synchronous switch into the scheduler context.
 *
 * Precondition: we are inside a thread context (current_context != nullptr).
 * The scheduler context ('thread_caller') runs the kernel reschedule and, when
 * this thread is later selected again, control returns here.
 */
static void switch_to_scheduler_context()
{
   CYROS_ASSERT(current_core.current_context); // Not inside a thread context
   CYROS_ASSERT(current_core.thread_caller);   // No scheduler context to resume
   current_core.thread_caller = context_handle(current_core.thread_caller.jump(nullptr).fctx);
}

/**
 * @brief Drain a deferred reschedule if the core is now at baseline priority.
 *
 * Baseline = inside a thread context, interrupt depth 0, preempt depth 0.
 * Called whenever either depth counter returns to zero.
 */
static void resolve_pending_reschedule_if_baseline()
{
   if (!current_core.reschedule_pending) return;

   // Baseline requires BOTH counters at zero...
   if (current_core.interrupt_disable_depth != 0) return;
   if (current_core.preempt_disable_depth   != 0) return;
   // ...and that we are inside a thread context to switch away from.
   if (!current_core.current_context) return;

   current_core.reschedule_pending = false;
   switch_to_scheduler_context();
}

/**
 * @brief Print the surrounding code given the source location
 */
static void print_formatted_context(char const* file, int target_line, int range = 2)
{
   // Colour Constants
   static constexpr auto CLR_RESET  = "\033[0m";
   static constexpr auto CLR_RED    = "\033[1;31m";
   static constexpr auto CLR_ORANGE = "\033[38;5;208m";

   std::ifstream fs(file);
   if (!fs.is_open()) return;

   std::string text;
   int current = 0;
   int start = (target_line - range > 0) ? target_line - range : 1;
   int end = target_line + range;

   while (std::getline(fs, text)) {
      current++;
      if (current >= start && current <= end) {
         std::printf("├ ");
         std::printf("%s%4d%s  ", CLR_ORANGE, current, CLR_RESET);
         if (current == target_line) {
            std::printf("%s>> %s%s\n", CLR_RED, text.c_str(), CLR_RESET);
         } else {
            std::printf("   %s\n", text.c_str());
         }
      }
      if (current > end) break;
   }
}


/* ============================================================================
 * Port Contract API
 * ----------------------------------------------------------------------------
 * Complete implementation of the contract:
 * ========================================================================= */

/* ----------------------------------------------------------------------------
 * Platform Initialisation
 * ------------------------------------------------------------------------- */

void cyros_port_init(cyros_port_reschedule_t reschedule_handler)
{
   global.reschedule_handler = reschedule_handler;
}


/* ----------------------------------------------------------------------------
 * SMP & Multi-Core Support
 *
 * Each pthread represents a simulated "core". Core 0 runs on the calling
 * thread, additional cores spawn as pthreads.
 * ------------------------------------------------------------------------- */

uint32_t cyros_port_get_core_id(void)
{
   // If no cores have been explicitly launched yet, then we must be on core0
   if (!global.cores_launched()) return 0;

   CYROS_ASSERT(current_core.core != nullptr);
   return current_core.core->core_id;
}
void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry)

{
   CYROS_ASSERT(cores_to_use > 0); // Invoking with 0 cores_to_use is invalid
   CYROS_ASSERT(cores_to_use <= CYROS_PORT_CORE_COUNT);

   global.cores = cpu_core_array(cores_to_use, entry);

   for (auto& core : global.cores) {
      // No need to spawn the first core/thread as that is assigned to this current calling core/thread
      if (core.core_id == 0) continue;

      pthread_create(
         &core.pthread,
         nullptr,
         +[](void* arg)-> void*
         {
            auto* init = static_cast<cpu_core*>(arg);
            current_core.core = init;

            // Enter the scheduler context
            init->start_scheduler();

            // On exit, we finish this OS-thread instance and core0's OS-thread can join with us
            return nullptr;
         },
         &core
      );
   }

   // core0 runs on calling thread
   auto& core0 = global.cores[0];
   current_core.core = &core0;

   // Enter the scheduler context for core0
   core0.start_scheduler();

   // When core0's scheduler context returns, join to any other active Core OS-thread
   for (auto& core : global.cores) {
      if (core.core_id == 0) continue;
      pthread_join(core.pthread, nullptr);
   }
   global.reset();
}

void cyros_port_send_reschedule_ipi(uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, global.cores.size());

   auto& core_poke = global.cores[core_id].core_poke;

   // Set the pending bit first (release) so the woken core sees it.
   core_poke.pending.store(true, std::memory_order_release);

   // Wake the core if it is blocked in idle().
   pthread_mutex_lock(&core_poke.mutex);
   pthread_cond_signal(&core_poke.cond_var);
   pthread_mutex_unlock(&core_poke.mutex);

   if (core_id == current_core.core->core_id && current_core.current_context) {
      // Targeting our own core from within a thread context: an IPI is a weak,
      // deferred-safe request - route through pend_reschedule(), not a forced
      // synchronous yield.
      cyros_port_pend_reschedule();
   }
}


/* ----------------------------------------------------------------------------
 * Interrupt Control - Simulated
 *
 * Tracks an interrupt-masking nesting depth. Blocks the hardware: while
 * non-zero, (simulated) interrupts cannot be delivered.
 * ------------------------------------------------------------------------- */

bool cyros_port_interrupts_enabled(void)
{
   return current_core.interrupt_disable_depth == 0;
}

cyros_mask_token_t cyros_port_irq_save(void)
{
   current_core.interrupt_disable_depth++;
   return 0; // cooperative port: no real mask state, token is inert
}

void cyros_port_irq_restore(cyros_mask_token_t token)
{
   (void)token;
   // Unwind one nesting level
   if (current_core.interrupt_disable_depth > 0) {
      current_core.interrupt_disable_depth--;
   }

   // Interrupt depth reaching 0 is one of the contract's safe points: if a
   // reschedule was pended while masked, resolve it (only fires if preemption
   // is also enabled).
   resolve_pending_reschedule_if_baseline();
}


/* ----------------------------------------------------------------------------
 * Preemption Control
 *
 * Tracks a preemption-disable nesting depth. Blocks the scheduler: while
 * non-zero, no context switch is performed on this core. Interrupts are
 * unaffected.
 * ------------------------------------------------------------------------- */

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   current_core.preempt_disable_depth++;
   return 0; // cooperative port: token is inert
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   (void)token;
   CYROS_ASSERT(current_core.preempt_disable_depth > 0); // unbalanced enable
   current_core.preempt_disable_depth--;

   // Preempt depth reaching 0 is one of the contract's safe points: if a
   // reschedule was pended while preemption was disabled, resolve it (only
   // fires if interrupts are also unmasked).
   resolve_pending_reschedule_if_baseline();
}

/* ----------------------------------------------------------------------------
 * Context Management & Switching
 * ------------------------------------------------------------------------- */

/**
 * @brief Entry point of a user thread's context.
 */
[[gnu::noreturn]]
static void thread_trampoline(boost::context::transfer_t entry_transfer)
{
   auto* context = static_cast<cyros_port_context*>(entry_transfer.data);

   current_core.thread_caller   = context_handle(entry_transfer.fctx);
   current_core.current_context = context;

   context->entry(context->arg); // Enter user code

   current_core.current_context = nullptr;

   // Final hand-back. The sentinel tells cyros_port_switch that this stack is
   // finished, so it stores an empty handle rather than one pointing at a dead
   // context. boost::context conveyed the same thing by returning an empty fiber.
   current_core.thread_caller.jump(thread_finished_signal);
   CYROS_PORT_UNREACHABLE(); // a finished stack is never resumed
}

void cyros_port_context_init(cyros_port_context_t* context,
                             void* stack_base,
                             size_t stack_size,
                             cyros_port_entry_t entry,
                             void* arg)
{
   global.active_contexts.fetch_add(1, std::memory_order_seq_cst);

   // Construct cyros_port_context_t in place
   ::new (context) cyros_port_context{
      .thread     = {},
      .stack_top  = static_cast<uint8_t*>(stack_base) + stack_size,
      .stack_size = stack_size,
      .entry      = entry,
      .arg        = arg,
   };

   // make_fcontext takes caller-owned memory directly, which is why the old
   // preallocated/stubbed-allocator dance is gone. stack_top is the high address:
   // stacks grow down.
   context->thread = context_handle(
      boost::context::make_fcontext(context->stack_top, context->stack_size, &thread_trampoline)
   );
}

void cyros_port_context_destroy(cyros_port_context_t* context)
{
   // Verify the thread context has run to completion
   CYROS_ASSERT(!context->thread); // Bug: destroying a live thread

   context->~cyros_port_context();
}

void cyros_port_switch(cyros_port_context_t* /*from*/, cyros_port_context_t* to)
{
   CYROS_ASSERT(to->thread); // No context to switch to

   current_core.current_context = to;

   // The first entry consumes `to` as the trampoline's argument; later resumes
   // land inside the thread's own jump() and ignore it.
   auto const back = to->thread.jump(to);

   to->thread = (back.data == thread_finished_signal) ? context_handle{}
                                                      : context_handle(back.fctx);

   current_core.current_context = nullptr;
}


void cyros_port_start_first(cyros_port_context_t* first)
{
   // Nothing special to be done on the first switch
   cyros_port_switch(nullptr, first);
}


/* ----------------------------------------------------------------------------
 * Reschedule Requests
 * ------------------------------------------------------------------------- */

void cyros_port_thread_yield(void)
{
   // Strong-guarantee, synchronous. Contract precondition: thread context at
   // baseline priority. Assert it - this port can observe all conditions.
   CYROS_ASSERT(current_core.current_context);            // must be a thread
   CYROS_ASSERT(current_core.interrupt_disable_depth == 0); // interrupts unmasked
   CYROS_ASSERT(current_core.preempt_disable_depth   == 0); // preemption enabled

   switch_to_scheduler_context();
}


void cyros_port_pend_reschedule(void)
{
   // Weak-guarantee, deferred-safe. Callable from any context.

   // Not inside a thread context (e.g. called from scheduler/idle context with
   // no current thread): nothing to switch away from. The cooperative pump in
   // start_scheduler() drives progress in that case.
   if (!current_core.current_context) return;

   const bool baseline = (current_core.interrupt_disable_depth == 0) &&
                         (current_core.preempt_disable_depth   == 0);

   if (baseline) {
      // The next safe point is now - resolve immediately. Clear any stale
      // pending flag; we are servicing it here.
      current_core.reschedule_pending = false;
      switch_to_scheduler_context();
   } else {
      // Kernel-masked: defer. The flag is drained at the next safe point -
      // whichever of irq_restore() / preempt_enable() leaves both depths at 0.
      current_core.reschedule_pending = true;
   }
}

void cyros_port_thread_exit(cyros_mask_token_t token)
{
   CYROS_ASSERT(current_core.preempt_disable_depth > 0); // coop port doesnt care if this is disabled, but it is asserting a kernel contract
   CYROS_ASSERT(global.active_contexts.load(std::memory_order_relaxed) != 0);
   global.active_contexts.fetch_sub(1, std::memory_order_seq_cst);

   cyros_port_preempt_enable(token);
}


/* ----------------------------------------------------------------------------
 * Thread-Local Storage
 * ------------------------------------------------------------------------- */

void cyros_port_set_tls_pointer(void* tls_base)
{
   current_core.tls_pointer = tls_base;
}

void* cyros_port_get_tls_pointer(void)
{
   return current_core.tls_pointer;
}


/* ----------------------------------------------------------------------------
 * CPU Hints & Idle
 * ------------------------------------------------------------------------- */

void cyros_port_cpu_relax(void)
{
   // CPU yield hint for busy-wait loops
#if defined(__x86_64__) || defined(__i386__)
   __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
   __asm__ __volatile__("yield");
#endif
}

void cyros_port_idle(void)
{
   // std::printf("(CORE %d) cyros_port_idle()\n", current_core.core->core_id);
   auto& core_poke = global.cores[current_core.core->core_id].core_poke;

   // Fast path: don't sleep if already pending.
   if (core_poke.pending.exchange(false, std::memory_order_acq_rel)) {
      return;
   }

   pthread_mutex_lock(&core_poke.mutex);

   // Re-check under lock (avoids missed wake if signal happens between fast-path and lock)
   while (!core_poke.pending.exchange(false, std::memory_order_acq_rel)) {
      pthread_cond_wait(&core_poke.cond_var, &core_poke.mutex);
   }

   pthread_mutex_unlock(&core_poke.mutex);
}


/* ----------------------------------------------------------------------------
 * Debug & Diagnostics
 * ------------------------------------------------------------------------- */

void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional)
{
   std::printf("KERNEL PANIC at %s:%d\n", file_optional, line_optional);
   print_formatted_context(file_optional, line_optional);
   std::printf("└ AUX1: 0x%lX, AUX2: 0x%lX\n", auxilary1, auxilary2);
   std::terminate();
}

void cyros_port_wait_for_debugger(void)
{
   volatile int pause = 1;
   printf("Attach GDB for PID: %d\n'set var pause = 0' to continue\n", getpid());

   while (pause) {
      usleep(1000);
   }
}

void cyros_port_breakpoint(void)
{
#if defined(__x86_64__) || defined(__i386__)
   __asm__ __volatile__("int3");
#elif defined(__aarch64__) || defined(__arm__)
   __builtin_trap();
#else
   raise(SIGTRAP);
#endif
}

void* cyros_port_get_stack_pointer(void)
{
   void* sp;
#if defined(__x86_64__)
   __asm__ __volatile__("mov %%rsp, %0" : "=r"(sp));
#elif defined(__i386__)
   __asm__ __volatile__("mov %%esp, %0" : "=r"(sp));
#elif defined(__aarch64__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#elif defined(__arm__)
   __asm__ __volatile__("mov %0, sp" : "=r"(sp));
#else
   int dummy;
   sp = &dummy;
#endif
   return sp;
}
