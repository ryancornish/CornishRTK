/**
 * @file port_linux_coop.cpp
 * @brief Linux simulation port, cooperative switching
 *
 * Simulates embedded behaviour (stack-based context switching) while running on
 * Linux for development and testing. Each pthread simulates one "core", and each
 * core owns a scheduler context that acts as the outer context for every thread
 * pinned to it. A thread reaching a reschedule point switches to that outer
 * context, so scheduling runs on its own stack.
 *
 * Reschedule model
 * ----------------
 * This port implements the two-operation reschedule contract from port.h:
 *
 *   cyros_port_thread_yield()    - strong guarantee, synchronous. Resumes the
 *                                  scheduler context immediately. Caller must be
 *                                  in thread context at baseline priority.
 *
 *   cyros_port_pend_reschedule() - weak guarantee, deferred-safe. At baseline
 *                                  priority it resolves now, the same
 *                                  synchronous switch. While the core is
 *                                  kernel-masked it sets a per-core
 *                                  "reschedule pending" flag instead.
 *
 * Baseline priority and the two depth counters
 * --------------------------------------------
 * "Kernel-masked" on this port means EITHER counter is non-zero:
 *   - interrupt_disable_depth : interrupt masking (Critical Sections).
 *   - preempt_disable_depth   : preemption disabling (Preemption Control).
 * A context switch may occur only at "baseline priority": inside a thread
 * context with BOTH counters at zero.
 *
 * Unlike the preempt port there is no OS mask to keep in step with the counters,
 * because nothing here is delivered asynchronously. The counters are the whole
 * mechanism, not a view of one.
 *
 * Resolving the pending flag
 * --------------------------
 * The flag is drained at whichever safe point is reached last, that is, whenever
 * a counter returns to zero and the OTHER counter is already zero:
 *   - cyros_port_irq_restore() reaching interrupt depth 0, and
 *   - cyros_port_preempt_enable() reaching preempt depth 0.
 * Both paths funnel through resolve_pending_reschedule_if_baseline(), which only
 * acts when the full baseline condition holds.
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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>


/* ============================================================================
 * Context Handles And The Transfer Protocol
 * ========================================================================= */

/**
 * @brief Handle to a suspended CPU context.
 *
 * A raw fcontext_t is a bare pointer that a jump does not invalidate, so reusing
 * a spent one is silent corruption. This wrapper makes that case loud: a jump
 * empties the handle, and every jump site assigns the returned handle back.
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

/* transfer_t::data protocol.
 *
 * A make_fcontext entry point receives its argument through the data pointer of
 * the FIRST jump into it. Nothing else is carried:
 *
 *   first entry into a thread      : the cyros_port_context*
 *   first entry into a scheduler   : the cpu_core*
 *   ordinary suspend, either way   : nullptr
 *
 * A thread entry cannot finish (see cyros_port_entry_t), so control only arrives
 * back at a jump site from a suspend and every resume stores the handle. */


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


/* ============================================================================
 * Internal Configuration
 * ========================================================================= */

/**
 * Per-core stack on which the kernel reschedule runs, separate from every thread
 * stack. Generously sized to hold the full reschedule call depth.
 */
static constexpr std::size_t scheduler_stack_size = 128 * 1024; // 128KB


/* ============================================================================
 * Internal State
 * ========================================================================= */

/**
 * @brief Simulated core. One pthread per core, holding only what another core
 *        needs in order to reach it.
 */
struct cpu_core
{
   pthread_t               pthread{}; // @note This is null/unused for core0
   uint32_t                core_id{}; // Index from 0... num cores
   cyros_port_core_entry_t entry{};

   /**
    * @brief Primitives to signal/communicate with other cores.
    */
   struct core_poke
   {
      pthread_mutex_t   mutex{};
      pthread_cond_t    cond_var{};
      std::atomic<bool> pending{false}; // Can be set by any core
      core_poke()  { pthread_mutex_init(&mutex, nullptr); pthread_cond_init(&cond_var, nullptr); }
      ~core_poke() { pthread_cond_destroy(&cond_var); pthread_mutex_destroy(&mutex); }
   } core_poke;
};

/**
 * @brief Dynamically-sized array/container wrapper for cpu_core's
 *
 * Using a std::vector _would_ be preferable, but impossible as a cpu_core is non-copyable
 */
class cpu_core_array
{
public:
   using iterator       = cpu_core*;
   using const_iterator = cpu_core const*;

   constexpr cpu_core_array() = default;
   explicit cpu_core_array(size_t count, cyros_port_core_entry_t core_entry)
      : cores(std::make_unique<cpu_core[]>(count)), count(count)
   {
      for (uint32_t i = 0; i < count; ++i) {
         cores[i].core_id = i;
         cores[i].entry   = core_entry;
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
   size_t                      count{0};
};

struct global_state
{
   std::atomic<bool>       shutdown_requested{false};
   std::atomic<uint32_t>   active_contexts{0};
   cyros_port_reschedule_t reschedule_handler{nullptr};
   cpu_core_array          cores;

   /**
    * @brief Have any cores been launched?
    */
   [[nodiscard]] bool cores_launched() const { return cores.size() > 0; }

   void reset()
   {
      shutdown_requested.store(false);
      reschedule_handler = nullptr;
      cores = {};
   }
};
static constinit global_state global;

/**
 * @brief Per-OS-thread (per-core) state.
 *
 * State control is owned solely by 'this' OS-thread. It cannot be accessed
 * by another core (enforced by thread_local). Initialised on pthread/core construction.
 * All cross-core data/state sharing is done via global_state.cores instead.
 */
struct current_core_state
{
   // Core's scheduler stack.
   // Layout: [ guard page (PROT_NONE) ][ scheduler_stack_size usable, grows down ]
   context_handle      scheduler_context;
   void*               scheduler_mapping{nullptr};
   std::size_t         scheduler_mapping_bytes{0};

   // This thread's core. An index into global.cores,
   // Defaults to 0: the bootstrap thread is core 0.
   std::uint32_t       core_id{0};

   // Non-null while executing inside a thread context on this core.
   cyros_port_context* current_context{nullptr};

   // The context the running thread came from, resumed to re-enter the scheduler.
   context_handle      thread_caller;

   // The outermost context, resumed to leave the scheduler when this core is done.
   context_handle      os_caller;

   // Interrupt-masking depth (Critical Sections). Masks the hardware.
   std::uint32_t       interrupt_disable_depth{0};

   // Preemption-disable depth (Preemption Control). Blocks the switch while the
   // simulated interrupt still flows.
   std::uint32_t       preempt_disable_depth{0};

   // Set by cyros_port_pend_reschedule() when it cannot resolve immediately,
   // that is, while the core is kernel-masked. Drained at the next safe point,
   // whichever of irq_restore() / preempt_enable() leaves BOTH depths at zero.
   bool                reschedule_pending{false};

   /**
    * @brief Map the guarded scheduler stack.
    */
   void allocate_scheduler_stack()
   {
      std::size_t const guard = guard_page_size();
      scheduler_mapping_bytes = guard + scheduler_stack_size;

      scheduler_mapping = mmap(nullptr, scheduler_mapping_bytes,
                               PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
      CYROS_ASSERT(scheduler_mapping != MAP_FAILED); // Out of memory for a scheduler stack

      int const guarded = mprotect(scheduler_mapping, guard, PROT_NONE);
      CYROS_ASSERT(guarded == 0); // Could not guard the scheduler stack
   }

   /**
    * @brief Release scheduler stack memory and teardown page guard
    */
   void free_scheduler_stack()
   {
      if (scheduler_mapping == nullptr) return;

      munmap(scheduler_mapping, scheduler_mapping_bytes);
      scheduler_mapping       = nullptr;
      scheduler_mapping_bytes = 0;
   }

   static std::size_t guard_page_size()
   {
      auto const page = sysconf(_SC_PAGESIZE);
      return page > 0 ? static_cast<std::size_t>(page) : 4096u;
   }
};
static thread_local constinit current_core_state current_core;


/* ============================================================================
 * Internal Helpers
 * ========================================================================= */

/* ----------------------------------------------------------------------------
 * Scheduler context (the outer context of every thread on this core)
 * ------------------------------------------------------------------------- */

/**
 * @brief Start the wind-down if only one idle thread per core is left.
 *
 * First caller wins the CAS and pokes every core, idempotent for the losers. The
 * poke is what gets a core out of cyros_port_idle(), where they are all parked
 * by then waiting on a condition variable nobody else will signal.
 */
static void request_shutdown_if_quiesced()
{
   if (global.active_contexts.load(std::memory_order_acquire) > global.cores.size()) return;

   bool expected = false;
   if (!global.shutdown_requested.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      return;
   }

   for (auto& c : global.cores) {
      cyros_port_send_reschedule_ipi(c.core_id);
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

   // Cooperative pump. The core that retires the last non-idle context latches
   // the flag and pokes everyone, so all cores leave on the same edge.
   while (!global.shutdown_requested.load(std::memory_order_acquire)) {
      global.reschedule_handler();
   }

   current_core.os_caller.jump(nullptr);
   CYROS_PORT_UNREACHABLE(); // the jump above does not come back
}

/**
 * @brief Enter this core's scheduler context. Returns when the core is done.
 */
static void start_scheduler()
{
   current_core.allocate_scheduler_stack();

   auto* const stack_top = static_cast<uint8_t*>(current_core.scheduler_mapping)
                         + current_core.scheduler_mapping_bytes;

   current_core.scheduler_context = context_handle(
      boost::context::make_fcontext(stack_top, scheduler_stack_size, &scheduler_trampoline)
   );

   // Enter the scheduler context. It jumps back here when this core is done,
   // and jump() has already emptied the handle by then.
   current_core.scheduler_context.jump(&global.cores[current_core.core_id]);

   // This thread owns the mapping, so this thread releases it.
   current_core.free_scheduler_stack();
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

/* ----------------------------------------------------------------------------
 * Deferred reschedule
 * ------------------------------------------------------------------------- */

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
   CYROS_ASSERT_OP(global.active_contexts.load(std::memory_order_relaxed), ==, 0u);
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
   // A stored per-thread fact, never inferred. See current_core_state::core_id.
   return current_core.core_id;
}

void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT(cores_to_use > 0); // Invoking with 0 cores_to_use is invalid
   CYROS_ASSERT(cores_to_use <= CYROS_PORT_CORE_COUNT);

   global.cores = cpu_core_array(cores_to_use, entry);

   // core0's OS thread is the caller and its TLS survives across kernel runs, so
   // reset it rather than leaving each field to be individually un-staled.
   current_core = current_core_state{};

   for (auto& core : global.cores) {
      // core0 is the calling thread, so it is not spawned here.
      if (core.core_id == 0) continue;

      pthread_create(
         &core.pthread,
         nullptr,
         +[](void* arg) -> void*
         {
            auto* init = static_cast<cpu_core*>(arg);
            current_core.core_id = init->core_id;

            // Runs the kernel entry for this core, which starts its first thread.
            start_scheduler();

            // On exit, we finish this OS-thread instance and core0's OS-thread can join with us
            return nullptr;
         },
         &core
      );
   }

   // core0 on the calling thread.
   auto& core0 = global.cores[0];
   current_core.core_id = core0.core_id;

   start_scheduler();

   // core0's scheduler context returns only at shutdown. Join the other cores,
   // which unwind the same way.
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

   if (core_id == current_core.core_id && current_core.current_context) {
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
   (void)token; // inert on this port, there is no mask to restore

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
   (void)token; // inert on this port, there is no mask to restore
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
 *
 * @note Must never return. A make_fcontext entry that returns is undefined
 *       behaviour
 */
[[gnu::noreturn]]
static void thread_trampoline(boost::context::transfer_t entry_transfer)
{
   auto* context = static_cast<cyros_port_context*>(entry_transfer.data);

   current_core.thread_caller   = context_handle(entry_transfer.fctx);
   current_core.current_context = context;

   context->entry(context->arg); // Enter user code, which does not come back

   CYROS_PORT_UNREACHABLE(); // Bug: a thread entry returned, see cyros_port_entry_t
}

void cyros_port_context_init(cyros_port_context_t* context,
                             void* stack_base,
                             size_t stack_size,
                             cyros_port_entry_t entry,
                             void* arg)
{
   global.active_contexts.fetch_add(1, std::memory_order_seq_cst);

   ::new (context) cyros_port_context{
      .thread     = {},
      .stack_top  = static_cast<uint8_t*>(stack_base) + stack_size,
      .stack_size = stack_size,
      .entry      = entry,
      .arg        = arg,
   };

   // make_fcontext takes caller-owned memory directly. stack_top is the high
   // address: stacks grow down.
   context->thread = context_handle(
      boost::context::make_fcontext(context->stack_top, context->stack_size, &thread_trampoline)
   );
}

void cyros_port_context_destroy(cyros_port_context_t* context)
{
   CYROS_ASSERT(global.active_contexts.load(std::memory_order_relaxed) != 0);

   // Drives the quiesce test in the pump. Safe to test here because the count
   // drops inside the arbiter, so a thread that has left entry() but is not yet
   // retired is still counted.
   global.active_contexts.fetch_sub(1, std::memory_order_seq_cst);

   // A system that never started cannot have quiesced.
   if (global.cores_launched()) request_shutdown_if_quiesced();

   // Abandoned, not unwound. An fcontext owns no memory and the stack is the
   // user's buffer, so a suspended one costs nothing to drop. Never unwind one,
   // that is what would need exceptions.
   context->thread = context_handle{};

   context->~cyros_port_context();
}

void cyros_port_switch(cyros_port_context_t* /*from*/, cyros_port_context_t* to)
{
   CYROS_ASSERT(to->thread); // No context to switch to

   current_core.current_context = to;

   // The first entry consumes `to` as the trampoline's argument; later resumes
   // land inside the thread's own jump() and ignore it.
   auto const back = to->thread.jump(to);

   // Always resumable: an entry never returns, so this is always a suspend.
   // Only cyros_port_context_destroy empties the handle.
   to->thread = context_handle(back.fctx);

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
   CYROS_ASSERT(current_core.current_context);              // must be a thread
   CYROS_ASSERT(current_core.interrupt_disable_depth == 0); // interrupts unmasked
   CYROS_ASSERT(current_core.preempt_disable_depth   == 0); // preemption enabled

   switch_to_scheduler_context();
}

void cyros_port_pend_reschedule(void)
{
   // Weak-guarantee, deferred-safe. No thread context means nothing to switch
   // away from, and the cooperative pump in scheduler_trampoline() drives
   // progress in that case.
   if (!current_core.current_context) return;

   bool const baseline = (current_core.interrupt_disable_depth == 0) &&
                         (current_core.preempt_disable_depth   == 0);

   if (baseline) {
      // The next safe point is now, so resolve immediately. Clear any stale
      // pending flag, we are servicing it here.
      current_core.reschedule_pending = false;
      switch_to_scheduler_context();
   } else {
      // Kernel-masked: defer. The flag is drained at the next safe point,
      // whichever of irq_restore() / preempt_enable() leaves both depths at 0.
      current_core.reschedule_pending = true;
   }
}


/* ----------------------------------------------------------------------------
 * Thread-Local Storage
 *
 * Absent (located in ../common/port_linux_common.cpp instead):
 * - cyros_port_set_tls_pointer()
 * - cyros_port_get_tls_pointer()
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * CPU Hints
 *
 * Absent (located in ../common/port_linux_common.cpp instead):
 * - cyros_port_cpu_relax()
 *
 * Parking a core stays here, because that is exactly where the two linux ports
 * differ: no signal can arrive, so a core waits on its own condition variable.
 * ------------------------------------------------------------------------- */

void cyros_port_idle(void)
{
   auto& core_poke = global.cores[current_core.core_id].core_poke;

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
 *
 * Absent (located in ../common/port_linux_common.cpp instead):
 * - cyros_port_system_error()
 * - cyros_port_wait_for_debugger()
 * - cyros_port_breakpoint()
 * - cyros_port_get_stack_pointer()
 * ------------------------------------------------------------------------- */
