/**
 * @file port_linux_preempt.cpp
 * @brief Linux simulation port using sigctx for genuine preemptive switching
 *
 * Where the boost.context port is cooperative end to end, this port exists to
 * exercise the one path that port structurally cannot reach: an asynchronous
 * signal interrupting a running thread mid-instruction and forcing a context
 * switch out of it. Each pthread still simulates one "core".
 *
 * One road into the scheduler
 * ---------------------------
 * Every reschedule on this port travels through a single signal. The preemption
 * tick (future), the cross-core IPI, the voluntary yield, and the deferred pend
 * all reduce to "make the reschedule signal land". sigctx_intercept captures the
 * interrupted context, hands it to on_reschedule(), and resumes whatever context
 * that handler returns. The handler runs the kernel scheduler, which ends in
 * cyros_port_switch(). That records the context to resume and returns, and the
 * interceptor performs the rt_sigreturn into it. So the kernel's reschedule
 * routine stays a normal call that unwinds, with no hidden non-local exit. The
 * signal handler is the analogue of PendSV running on the handler stack.
 *
 * Synchronous yield (first-step simplification)
 * ---------------------------------------------
 * cyros_port_thread_yield() raises the reschedule signal against itself. At
 * baseline the signal is delivered before pthread_kill() returns, so the yield
 * still gets its strong synchronous round-trip. This pays a signal round-trip
 * per yield. A direct synchronous sigctx swap would be faster and can replace it
 * later once the preemptive path is proven.
 *
 * Masking model
 * -------------
 * Interrupt-disable and preempt-disable gate the SAME reschedule signal here,
 * because the reschedule handler does nothing beyond the switch it is already
 * deferring. So blocking the signal under either depth is correct, and the
 * kernel's pending bit doubles as the deferred-reschedule flag. There is no
 * separate software pending flag.
 *
 * The two depths still mean different things, and the difference becomes visible
 * when the timer port lands. A timer signal will be masked by interrupt-disable
 * (its ISR must not run) and left unmasked by preempt-disable (its ISR runs, and
 * only the switch it requests is deferred). The block helpers below are where a
 * timer signal joins the interrupt path and stays out of the preempt path.
 *
 * The two depths are the state and the OS mask is a derived view of them, so
 * every phase that masks an owned signal, including both signal handlers and the
 * pre-kernel dormant phase, is expressed as a depth. See "Signal mask model"
 * below.
 *
 * Lifetime hazard the whole port turns on
 * ---------------------------------------
 * The captured context the handler receives, and its FP buffer, live on the
 * handler stack that sigctx_intercept carves per invocation. They are valid only
 * for that one handler call. So the outgoing thread's state is relocated into its
 * TCB with sigctx_copy() before any other context is resumed, and the resumed
 * context is always a durable TCB context.
 */

#include <cyros/port/port.h>

#include "port_linux_preempt_internal.hpp"

#include <sigctx/sigctx.h>
#include <sigctx/sigctx_intercept.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <pthread.h>
#include <ranges>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>


/* ============================================================================
 * Port Context Structure
 * ========================================================================= */

struct cyros_port_context
{
   sigctx_inl_t         sctx;        // resumable signal-frame context, FP inline
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
 * The signal that drives every reschedule on this port. SIGURG is chosen
 * because typical applications do not use it, so it does not collide with
 * ordinary signal handling. The port owns this signal's disposition entirely.
 */
static constexpr int preempt_signo = SIGURG;

/**
 * The signal the time-driver port raises to deliver a timer interrupt. It MUST
 * match timer_signo in port_time_linux_preempt.cpp. Interrupt-disable masks it,
 * preempt-disable does not, which is the whole interrupt-versus-preempt split.
 * SIGRTMIN is not a constant expression, so this is a runtime-initialised const.
 */
static const int timer_signo = SIGRTMIN;

/**
 * Per-core stack on which the interceptor trampoline, the capture relocation,
 * and the kernel reschedule all run. This is distinct from the library-internal
 * altstack where the kernel lays the raw signal frame. Generously sized to hold
 * a captured context, its FP area, and the full reschedule call depth.
 */
static constexpr std::size_t handler_stack_size = 128 * 1024; // 128KB


/* ============================================================================
 * Internal State
 * ========================================================================= */

/**
 * @brief Simulated core. One pthread per core, holding only what another core
 *        needs in order to reach it.
 */
struct cpu_core
{
   pthread_t               pthread{}; // core0 records pthread_self() here too
   uint32_t                core_id{};
   cyros_port_core_entry_t entry{};

   /* Brought-up flag for the IPI guard in cyros_port_send_reschedule_ipi. The
    * pthread handle is written by the spawning thread and read by any core
    * wanting to signal this one, and an opaque pthread_t is neither atomic nor
    * portably comparable, so the cross-core question "can this core receive a
    * signal yet" gets its own atomic rather than being inferred from the
    * handle. Always stored after the handle is recorded, so an acquire load
    * that observes it also observes a valid handle. */
   std::atomic<bool>       started{false};
};

struct global_state
{
   std::atomic<bool>       shutdown_requested{false};
   std::atomic<uint32_t>   active_contexts{0};
   cyros_port_reschedule_t reschedule_handler{nullptr};
   std::vector<cpu_core> cores;

   /**
    * @brief Have any cores been launched?
    */
   [[nodiscard]] bool cores_launched() const { return cores.size() > 0; }

   void reset()
   {
      shutdown_requested.store(false);
      active_contexts.store(0);
      reschedule_handler = nullptr;
      cores.clear(); // not '= {}', the started atomic makes cpu_core non-copyable
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
   // Core's scheduler stack setup for sigctx.
   // Layout: [ guard page (PROT_NONE) ][ handler_stack_bytes usable, grows down ]
   std::uint8_t*       handler_stack{nullptr};
   std::size_t         handler_stack_bytes{0};
   void*               stack_mapping{nullptr};
   std::size_t         stack_mapping_bytes{0};

   // Captured at first signal delivery. Resuming it unwinds through
   // cyros_port_start_first() so the owning OS-thread can be joined.
   sigctx_inl_t        scheduler_ctx{};

   // This thread's core. An index into global.cores,
   // Defaults to 0: the bootstrap thread is core 0.
   std::uint32_t       core_id{0};

   // Non-null while executing inside a thread context on this core.
   cyros_port_context* current_context{nullptr};

   // The live captured (paused) frame on the handler stack, set by on_reschedule() and
   // later consumed by the following cyros_port_switch(). The lifetime of this
   // pointer (and the pointed-to context object) is only valid for this duration.
   // This technique is necessary for interopt with the sigctx interceptor routine.
   sigctx_ucontext_t*  paused_uc{nullptr};

   // The context cyros_port_switch() chose, read back by on_reschedule() and
   // returned to the sigctx interceptor to resume. When set, always points to a TCB's
   // context frame.
   sigctx_ucontext_t*  resume_uc{nullptr};

   // First signal delivery on this core takes the bring-up branch.
   bool                bootstrapping{false};
   cyros_port_context* first_ctx{nullptr};

   // The outgoing thread is dead, so the next switch must not save it back.
   bool                discard_outgoing{false};

   // Interrupt-masking depth (Critical Sections). Masks the hardware.
   std::uint32_t       interrupt_disable_depth{0};

   // Preemption-disable depth (Preemption Control). Blocks the switch while the
   // simulated interrupt still flows.
   std::uint32_t       preempt_disable_depth{0};

   // Re-entrancy guard for assert_mask_matches_depths(). See its comment.
   bool                mask_check_active{false};

   /**
    * @brief Map the guarded handler stack.
    * @return 0 on success, or a negative errno.
    */
   int allocate_handler_stack()
   {
      if (handler_stack != nullptr) return 0; // Already mapped (TODO: Should this be an assert?)

      std::size_t const guard = guard_page_size();
      stack_mapping_bytes = guard + handler_stack_size;

      stack_mapping = mmap(nullptr, stack_mapping_bytes,
                           PROT_READ | PROT_WRITE,
                           MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
      if (stack_mapping == MAP_FAILED) {
         stack_mapping       = nullptr;
         stack_mapping_bytes = 0;
         return -errno;
      }

      if (mprotect(stack_mapping, guard, PROT_NONE) != 0) {
         int const err = -errno;
         munmap(stack_mapping, stack_mapping_bytes);
         stack_mapping       = nullptr;
         stack_mapping_bytes = 0;
         return err;
      }

      handler_stack       = static_cast<std::uint8_t*>(stack_mapping) + guard;
      handler_stack_bytes = handler_stack_size;
      return 0;
   }

   /**
    * @brief Release handler stack memory and teardown page guard
    */
   void free_handler_stack()
   {
      if (stack_mapping == nullptr) return;

      munmap(stack_mapping, stack_mapping_bytes);
      stack_mapping       = nullptr;
      stack_mapping_bytes = 0;
      handler_stack       = nullptr;
      handler_stack_bytes = 0;
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
 * Signal mask model
 *
 * The two depth counters are the state. The OS mask is a DERIVED VIEW of them,
 * recomputed from scratch on every transition and never edited from remembered
 * history. That is what makes it idempotent and independent of the order in which
 * overlapping regions open and close.
 *
 * Why not the usual save/restore token, where a disable returns what it closed and
 * the enable reopens exactly that? Because interrupt- and preempt-disable both
 * gate preempt_signo, so one resource has two candidate owners. Each deferred to
 * the other and neither transferred the obligation, which left the signal blocked
 * with nobody responsible:
 *
 *   irq_save     -> owns the reopen
 *   preempt_disable -> finds it already blocked, so disclaims the reopen
 *   irq_restore  -> declines, correctly, because a preempt region is still live
 *   preempt_enable -> declines, because its token says it never closed it
 *
 * Strict nesting worked. Overlap leaked, in both orders. Swapping to an absolute
 * SIG_SETMASK of the saved value does not help either: it unblocks preempt_signo
 * while the preempt region is still live, which is worse. The save/restore idiom
 * is only conditionally correct, requiring strictly LIFO nesting, and cyros
 * breaks that deliberately because thread_launcher hands its preempt token to
 * cyros_port_thread_exit() to release later.
 *
 * A real target has no such problem: interrupt-disable is PRIMASK and
 * preempt-disable is BASEPRI or a software scheduler lock, so they are DISTINCT
 * resources with nothing to arbitrate. Only here does one mechanism, the signal
 * mask, stand in for both. So the tokens stay in the port contract for the targets
 * that want them, and are simply UNUSED here. Do not reintroduce a decision based
 * on one, and note they cannot even serve as a cross-check: asserting that a
 * restore's token matches what the depths call for fires on legitimate overlap.
 * ------------------------------------------------------------------------- */
enum : cyros_mask_token_t
{
   mask_token_reschedule = 0x1u, // reschedule (PendSV analogue) was deliverable at disable
   mask_token_timer      = 0x2u, // timer interrupt was deliverable at disable
};

/**
 * @brief A signal this port owns, and which depth gates it.
 *
 * Interrupt-disable blocks everything here, because it is the superset.
 * Preempt-disable blocks only what is marked, so an ISR still runs under it and
 * only the switch it requests is deferred.
 *
 * THIS TABLE IS THE ONLY EXTENSION POINT. A new interrupt source is one row plus
 * one token bit above. Nothing else in the model enumerates signals.
 */
struct owned_signal
{
   int                signo;
   cyros_mask_token_t token_bit;
   bool               gated_by_preempt;
};

static std::array const owned_signals = std::to_array<owned_signal>({
   { .signo=preempt_signo, .token_bit=mask_token_reschedule, .gated_by_preempt=true  },
   { .signo=timer_signo,   .token_bit=mask_token_timer,      .gated_by_preempt=false },
});

/**
 * @brief Does the given depth state call for @p s to be blocked?
 *
 * Takes the depths rather than reading them, so a raise path can ask what the
 * mask will need to be a moment before it needs it. That is the whole reason
 * this is parameterised: see the ordering rule on block_for().
 *
 * Monotone non-decreasing in both depths, which is what makes a raise able to
 * add blocks and never need to remove one.
 */
static bool should_block(owned_signal const& s, unsigned interrupt_depth, unsigned preempt_depth)
{
   if (interrupt_depth > 0) return true;
   return s.gated_by_preempt && preempt_depth > 0;
}

/**
 * @brief Does this core's CURRENT depth state call for @p s to be blocked?
 */
static bool should_block(owned_signal const& s)
{
   return should_block(s, current_core.interrupt_disable_depth,
                       current_core.preempt_disable_depth);
}

/**
 * @brief Tighten the mask to what @p interrupt_depth / @p preempt_depth call for.
 *
 * The RAISE half of the mask model, and the counterpart to apply_mask(). Callers
 * pass the depths they are ABOUT TO HAVE, then raise them.
 *
 * Only ever adds blocks, because should_block() is monotone in both depths, so
 * a raise can never need an unblock. That is why one SIG_BLOCK is the whole
 * operation, and why apply_mask() is not needed afterwards: the mask this
 * installs already IS what apply_mask() would derive once the depth is raised.
 *
 * ORDERING RULE, and the reason this function exists at all. The mask may be
 * MORE restrictive than the depths for an instant, never less. So:
 *
 *    raise:   tighten the mask, THEN raise the depth   (here)
 *    lower:   lower the depth,  THEN loosen the mask   (apply_mask, restores)
 *
 * Both leave the same safe skew, a moment where a signal is blocked that the
 * counters do not yet or no longer require. The opposite skew is a live defect:
 * `depth++` followed by apply_mask() leaves an interval where the counters say
 * the region is protected and the signal is still deliverable, and a reschedule
 * landing there captures the thread with the depth raised and its frame's mask
 * open. cross-core-defects.md 8.7d has the probe output, p=1 at handler entry
 * with frame_blk=0, and 8.7e has why the naive reorder cannot fix it.
 */
static void block_for(unsigned interrupt_depth, unsigned preempt_depth)
{
   sigset_t block;
   sigemptyset(&block);

   bool any = false;
   for (auto const& s : owned_signals) {
      if (should_block(s, interrupt_depth, preempt_depth)) {
         sigaddset(&block, s.signo);
         any = true;
      }
   }

   if (any) pthread_sigmask(SIG_BLOCK, &block, nullptr);
}

/**
 * @brief Bring the OS mask in line with the depth counters.
 *
 * Absolute in SEMANTICS: the result is a function of the counters alone, so
 * applying it twice is applying it once. Differential in MECHANISM, because
 * SIG_BLOCK and SIG_UNBLOCK preserve the signals this port does not own without
 * having to read the mask back first. Blocks before unblocking so no owned signal
 * is transiently deliverable, and skips a syscall for an empty set so a
 * transition still costs the one call it always did.
 *
 * The LOWERING half of the model. Raising paths use block_for() instead, which
 * can answer for a depth state that is not the current one.
 */
static void apply_mask()
{
   sigset_t block;
   sigemptyset(&block);
   sigset_t unblock;
   sigemptyset(&unblock);

   bool any_block   = false;
   bool any_unblock = false;

   for (auto const& s : owned_signals) {
      if (should_block(s)) {
         sigaddset(&block, s.signo);
         any_block = true;
      } else {
         sigaddset(&unblock, s.signo);
         any_unblock = true;
      }
   }

   if (any_block)   pthread_sigmask(SIG_BLOCK, &block, nullptr);
   if (any_unblock) pthread_sigmask(SIG_UNBLOCK, &unblock, nullptr); // delivers anything pended
}

/**
 * @brief The token a disable hands back, describing what was deliverable.
 *
 * Derived from the counters rather than read back from the OS, which is both a
 * syscall cheaper and honest: the mask is a view of the counters, so reading it
 * would only tell us what we already know. Call BEFORE raising a depth.
 */
static cyros_mask_token_t deliverable_token()
{
   cyros_mask_token_t token = 0;
   for (auto const& s : owned_signals) {
      if (!should_block(s)) token |= s.token_bit;
   }
   return token;
}

/**
 * @brief Assert the OS mask matches what the depths call for.
 *
 * Total over its domain rather than best-effort. The four public disable/enable
 * entry points are the only places cyros changes the mask, so checking there
 * covers every transition it performs. A handler's two boundaries are outside the
 * domain by construction: at both, the mask is installed by the signal machinery
 * and not by us, and no disable/enable can run in either window. See
 * cyros::port::isr_region.
 */
static void assert_mask_matches_depths()
{
#if CYROS_PORT_DEBUG_MODE
   // Guards the debugger path in cyros_port_system_error(). With
   // cyros_port_wait_for_debugger() enabled, a failure here reports through it,
   // it takes a critical section, and control lands straight back in this check.
   // The first failure would then recurse until the stack dies, and the SEGV
   // would bury the diagnostic that named it, precisely when someone is trying to
   // debug. Never cleared on the failing path: we are on our way out.
   if (current_core.mask_check_active) return;
   current_core.mask_check_active = true;

   sigset_t live;
   sigemptyset(&live);
   pthread_sigmask(SIG_BLOCK, nullptr, &live);

   for (auto const& s : owned_signals) {
      CYROS_ASSERT_OP(sigismember(&live, s.signo) != 0, ==, should_block(s));
   }

   current_core.mask_check_active = false;
#endif
}

/**
 * @brief Enter or leave the dormant region on this OS thread.
 *
 * Before a core starts its first thread, and again once its bring-up context is
 * resumed at shutdown, the OS thread is not running cyros threads and must not
 * take a reschedule. That is an interrupt-disabled state, so it is spelled with
 * the counter rather than a bare pthread_sigmask. Spelling every such phase this
 * way is what lets assert_mask_matches_depths() be unconditional instead of
 * riddled with exceptions.
 */
static void enter_dormant_region()
{
   // A raise, so it obeys the same ordering rule as the public raise paths:
   // tighten first, then raise into it. Every call site today happens to arrive
   // with the mask already at least this restrictive (a spawned core inherits
   // the dormant mask, and the shutdown resume is left blocked), so the old
   // raise-then-apply order was safe here incidentally rather than by the rule.
   // Spelling it the same way as the others removes a trap for the next call
   // site, which may not arrive already masked.
   block_for(current_core.interrupt_disable_depth + 1,
             current_core.preempt_disable_depth);
   ++current_core.interrupt_disable_depth;
   assert_mask_matches_depths();
}

static void leave_dormant_region()
{
   CYROS_ASSERT(current_core.interrupt_disable_depth > 0); // not dormant
   current_core.interrupt_disable_depth--;

   // This apply is a CONTROL-TRANSFER point, not just a mask edit: its unblock
   // delivers the reschedule pended while dormant, and cyros_port_start_first()
   // never returns from it under normal flow. The bring-up context is captured
   // inside this very call, so at shutdown execution resumes here in a different
   // phase, with the mask the shutdown branch chose rather than the one these
   // depths call for. Do not assert the invariant after it. start_first()'s
   // matching enter_dormant_region() re-establishes and checks it instead.
   apply_mask();
}


/* ----------------------------------------------------------------------------
 * Interrupt-handler regions
 *
 * Both handlers in this port are entered with exactly the owned signals blocked,
 * which is precisely the mask interrupt-disable calls for: the reschedule
 * interception because sigctx holds the trigger signal down for the whole
 * interception and ORs in block_extra, the timer ISR because delivery blocks its
 * own signal and sa_mask adds the reschedule. So both ARE interrupt-disabled
 * regions rather than merely analogous to one, and both say so with the counter.
 * That is why the model needs no handler special case.
 *
 * Both calls move the counter WITHOUT applying the mask, because at both ends the
 * machinery installs it: delivery on the way in, the sigreturn on the way out.
 * Neither can be atomic with a store to a counter, so the two disagree for the
 * instant between them. Safe for a checkable reason rather than by luck, see
 * assert_mask_matches_depths().
 * ------------------------------------------------------------------------- */

namespace cyros::port
{

void isr_region_enter()
{
   current_core.interrupt_disable_depth++;
}

void isr_region_leave()
{
   CYROS_ASSERT(current_core.interrupt_disable_depth > 0); // handler region underflowed
   current_core.interrupt_disable_depth--;
}

} // namespace cyros::port


/* ----------------------------------------------------------------------------
 * Reschedule handler (the scheduler context for this port)
 * ------------------------------------------------------------------------- */


/**
 * @brief Called by the interceptor with the just-paused context. Returns the
 *        context to resume.
 *
 * Guaranteed to be entered at baseline: the reschedule signal is only ever
 * delivered when both depth counters are zero. The handler body then runs as an
 * interrupt-disabled region, see cyros::port::isr_region.
 */
static sigctx_ucontext_t* on_reschedule(sigctx_ucontext_t* paused, void*)
{
   if (!global.cores_launched()) return paused; // Delivery after kernel teardown, ignore

   // Covers every return path below, so the depth is balanced whichever branch
   // chooses the context to resume.
   cyros::port::isr_region const in_handler;

   bool const bootstrapping = current_core.bootstrapping;

   if (bootstrapping) {
      current_core.bootstrapping = false;
      sigctx_copy(/*dst=*/ &current_core.scheduler_ctx.uc, current_core.scheduler_ctx.fpstate,
                  sizeof(current_core.scheduler_ctx.fpstate),
                  /*src=*/ paused);
   }

   // The system has quiesced to one idle thread per core.
   // Resume the bring-up point so this OS thread unwinds and
   // can be joined.
   if (global.shutdown_requested.load(std::memory_order_acquire)) {
      sigaddset(&current_core.scheduler_ctx.uc.uc_sigmask, preempt_signo);
      return &current_core.scheduler_ctx.uc;
   }

   if (bootstrapping) {
      current_core.current_context = current_core.first_ctx;
      return &current_core.first_ctx->sctx.uc;
   }

   // Drive the core-local scheduler. It ends in cyros_port_switch(), which
   // records the context to resume in resume_uc and returns normally, so the
   // kernel reschedule routine unwinds like any other call. Clear resume_uc
   // first so the assert below catches a scheduler path that failed to switch
   // rather than passing on a stale value.
   current_core.paused_uc = paused;
   current_core.resume_uc = nullptr;
   global.reschedule_handler();
   current_core.paused_uc = nullptr;

   CYROS_ASSERT(current_core.resume_uc); // scheduler returned without choosing a context
   return current_core.resume_uc;
}

/**
 * @brief Install the interceptor on the calling core. sigaltstack is per-thread,
 *        so each core's OS thread must call this once before running its entry.
 */
static int install_interceptor()
{
   int const mapped = current_core.allocate_handler_stack();
   if (mapped != 0) return mapped;

   // Keep the timer signal masked during reschedule
   sigset_t extra;
   sigemptyset(&extra);
   sigaddset(&extra, timer_signo);

   sigctx_intercept_cfg cfg{
      .signo       = preempt_signo,
      .handler_sp  = current_core.handler_stack,
      .handler_ss  = current_core.handler_stack_bytes,
      .handler     = on_reschedule,
      .arg         = nullptr, // the handler reads current_core, nothing to pass
      .block_extra = &extra,
   };
   return sigctx_intercept_install(&cfg);
}

/* ----------------------------------------------------------------------------
 * Diagnostics formatting
 * ------------------------------------------------------------------------- */


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
   // A stored per-thread fact, never inferred. See current_core_state::core_id.
   return current_core.core_id;
}

void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry)
{
   CYROS_ASSERT(cores_to_use > 0);
   CYROS_ASSERT(cores_to_use <= CYROS_PORT_CORE_COUNT);

   // A preempted thread's full FP state is relocated into its TCB context, whose
   // inline buffer is bounded by SIGCTX_FPSTATE_CAPACITY. If this machine's XSAVE
   // area is larger, that relocation would quietly demote to a legacy frame and
   // drop vector state across a preemption, so refuse to run rather than corrupt
   // registers.
   uint32_t fp_size = sigctx_fpstate_size();
   CYROS_ASSERT(fp_size <= SIGCTX_FPSTATE_CAPACITY); // raise capacity or use a dyn context

   global.cores = std::vector<cpu_core>(cores_to_use);
   for (auto const [index, core] : std::views::enumerate(global.cores)) {
      core.core_id = index;
      core.entry = entry;
   }

   // core0 runs on this calling OS thread. Record its handle so a cross-core IPI
   // can target it the same way as any spawned core.
   global.cores[0].pthread = pthread_self();
   global.cores[0].started.store(true, std::memory_order_release);

   // Prevent IPIs from initialized cores to uninitialized/initializing cores.
   // This avoids a premature reschedule before the target core is ready.
   // The block is removed when the target core starts its first thread.
   // core0's OS thread is the caller and its TLS survives across kernel runs, so
   // reset it rather than leaving each field to be individually un-staled. This
   // is what lets the dormant region below be a plain enter: a leftover depth from
   // a previous run would otherwise stack a second level, start_first()'s leave
   // would not reach zero, and the core would silently never schedule.
   current_core = current_core_state{};

   enter_dormant_region();

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

            // pthread_create inherits the creating thread's signal mask, which
            // is the dormant mask set above, but this thread's TLS is fresh and
            // its counters start at zero. Record the state we were born into.
            enter_dormant_region();

            int rc = install_interceptor();
            CYROS_ASSERT(rc == 0); // interceptor failed to install on this core

            // Runs the kernel entry for this core, which starts its first thread.
            init->entry();

            // Mirror of the allocation above: this thread owns the mapping, so
            // this thread releases it, once its core entry has unwound.
            current_core.free_handler_stack();
            return nullptr;
         },
         &core
      );
      core.started.store(true, std::memory_order_release);
   }

   // core0 on the calling thread.
   auto& core0 = global.cores[0];
   current_core.core_id = core0.core_id;

   int rc = install_interceptor();
   CYROS_ASSERT(rc == 0); // Interceptor failed to install on core0

   core0.entry();
   current_core.free_handler_stack(); // core0's thread owns its mapping too

   sigset_t now;
   pthread_sigmask(SIG_BLOCK, /*set=*/nullptr, &now);
   CYROS_ASSERT(sigismember(&now, preempt_signo)); // Shutdown unwind should have left this blocked

   // core0's entry returns only once its bring-up context has been resumed at
   // shutdown. Join the other cores, which unwind the same way.
   for (auto& core : global.cores) {
      if (core.core_id == 0) continue;
      pthread_join(core.pthread, nullptr);
   }

   // core0 has unwound and is no longer a scheduling participant. Any preempt_signo
   // still pending on this thread is a stale IPI or self-signal from teardown and
   // must not run on_reschedule against a reset kernel. Drain it while masked.
   sigset_t pend;
   sigpending(&pend);
   if (sigismember(&pend, preempt_signo)) {
      int sig;
      sigset_t only;
      sigemptyset(&only);
      sigaddset(&only, preempt_signo);
      sigwait(&only, &sig); // Consume the pending instance without running the handler
   }

   global.reset();
}

void cyros_port_send_reschedule_ipi(uint32_t core_id)
{
   CYROS_ASSERT_OP(core_id, <, global.cores.size());

   auto& target = global.cores[core_id];
   if (!target.started.load(std::memory_order_acquire)) {
      // Signalling a core that has not been brought up yet is dropped.
      // Allowed to be lossy because when the core is first brought up, it will
      // see at first-pick whatever this IPI attempt was trying to communicate to it.
      return;
   }

   // Targeting the signal at a core is the whole IPI. A core at baseline takes
   // it now, a masked core takes it on unmask, an idle core parked in
   // sigsuspend wakes and reschedules. Self-targeting works the same way.
   pthread_kill(target.pthread, preempt_signo);
}


/* ----------------------------------------------------------------------------
 * Interrupt Control
 *
 * Masks the hardware: while raised, neither the reschedule signal nor the timer
 * signal can be delivered to this core.
 * ------------------------------------------------------------------------- */

bool cyros_port_interrupts_enabled(void)
{
   return current_core.interrupt_disable_depth == 0;
}

cyros_mask_token_t cyros_port_irq_save(void)
{
   assert_mask_matches_depths();

   // Read the prior state before changing it, as PRIMASK would be. Informational
   // only on this port: the depth raised below is what decides the mask.
   cyros_mask_token_t const token = deliverable_token();

   // Tighten to what the raised depth calls for, THEN raise into it. Do not
   // fold the two together: block_for(..., ++depth) sequences the increment
   // BEFORE the syscall and re-opens the very window this closes.
   block_for(current_core.interrupt_disable_depth + 1,
             current_core.preempt_disable_depth);
   ++current_core.interrupt_disable_depth;
   return token;
}

void cyros_port_irq_restore(cyros_mask_token_t token)
{
   (void)token; // unusable on this port, see "Mask tokens" above
   CYROS_ASSERT(current_core.interrupt_disable_depth > 0); // unbalanced restore

   current_core.interrupt_disable_depth--;
   apply_mask();
   assert_mask_matches_depths();
}


/* ----------------------------------------------------------------------------
 * Preemption Control
 *
 * Blocks the switch but leaves interrupt signals free to run. Preempt-disable
 * masks only the reschedule signal, so a timer (or any interrupt) still fires
 * under it and only the reschedule that interrupt requests is deferred. That is
 * the difference from interrupt control, which masks both.
 * ------------------------------------------------------------------------- */

cyros_mask_token_t cyros_port_preempt_disable(void)
{
   assert_mask_matches_depths();

   cyros_mask_token_t const token = deliverable_token();

   // Tighten first, then raise into it. See the ordering rule on block_for,
   // and note the increment must stay a separate statement.
   block_for(current_core.interrupt_disable_depth,
             current_core.preempt_disable_depth + 1);
   ++current_core.preempt_disable_depth;
   return token;
}

void cyros_port_preempt_enable(cyros_mask_token_t token)
{
   (void)token; // unusable on this port, see "Mask tokens" above
   CYROS_ASSERT(current_core.preempt_disable_depth > 0); // unbalanced enable

   current_core.preempt_disable_depth--;
   apply_mask();
   assert_mask_matches_depths();
}


/* ----------------------------------------------------------------------------
 * Context Management & Switching
 * ------------------------------------------------------------------------- */

void cyros_port_context_init(cyros_port_context_t* context,
                             void* stack_base,
                             size_t stack_size,
                             cyros_port_entry_t entry,
                             void* arg)
{
   global.active_contexts.fetch_add(1, std::memory_order_seq_cst);

   ::new (context) cyros_port_context{
      .sctx       = {},
      .stack_top  = static_cast<uint8_t*>(stack_base) + stack_size,
      .stack_size = stack_size,
      .entry      = entry,
      .arg        = arg,
   };

   // Synthesize a resumable signal-frame context that enters entry(arg) on the
   // supplied stack. The FP area lives inline in this TCB context, so it travels
   // with the context and the self-pointer stays valid as long as the TCB does.
   sigctx_create(&context->sctx.uc,
                 context->sctx.fpstate, sizeof(context->sctx.fpstate),
                 stack_base, stack_size,
                 entry, arg);
}

void cyros_port_context_destroy(cyros_port_context_t* context)
{
   context->~cyros_port_context();
}

void cyros_port_switch(cyros_port_context_t* from, cyros_port_context_t* to)
{
   CYROS_ASSERT(to);
   CYROS_ASSERT(current_core.paused_uc); // a switch is only ever driven from on_reschedule

   if (from && !current_core.discard_outgoing) {
      // Relocate the captured frame into the outgoing TCB while paused_uc still
      // points at live handler-stack storage. This is what lets the thread
      // resume later, after this handler invocation is gone.
      sigctx_copy(/*dst=*/ &from->sctx.uc, from->sctx.fpstate, sizeof(from->sctx.fpstate),
                  /*src=*/ current_core.paused_uc);
   }

   current_core.discard_outgoing = false;
   current_core.paused_uc        = nullptr;
   current_core.current_context  = to;

   // Record the context to resume and return. on_reschedule() hands it back to
   // the interceptor, which performs the rt_sigreturn from the trampoline. This
   // keeps the kernel reschedule routine a plain returning call.
   current_core.resume_uc = &to->sctx.uc;
}

void cyros_port_start_first(cyros_port_context_t* first)
{
   current_core.first_ctx        = first;
   current_core.bootstrapping    = true;

   // Pend on_reschedule() to capture this bring-up point and jump into the
   // first thread.
   pthread_kill(pthread_self(), preempt_signo);

   // This core starts scheduling now, so leave the dormant region. Our
   // self-trigger is coalesced with any earlier IPI requests to us, so on unblock
   // one and only one reschedule interrupt occurs.
   leave_dormant_region();

   // Control returns here only at core-shutdown, when the captured scheduler_ctx
   // is resumed. That context carries preempt_signo blocked (added by the
   // shutdown branch of on_reschedule), so the mask is already dormant and this
   // only brings the depth back into agreement with it before we unwind and are
   // joined. Both halves of the handoff are therefore spelled the same way.
   enter_dormant_region();
}


/* ----------------------------------------------------------------------------
 * Reschedule Requests
 * ------------------------------------------------------------------------- */

void cyros_port_thread_yield(void)
{
   // Strong-guarantee, synchronous. Contract precondition: thread context at
   // baseline priority.
   CYROS_ASSERT(current_core.current_context);              // must be a thread
   CYROS_ASSERT(current_core.interrupt_disable_depth == 0); // interrupts unmasked
   CYROS_ASSERT(current_core.preempt_disable_depth   == 0); // preemption enabled

   // At baseline the signal is delivered before this returns. The handler
   // captures this point and switches us out, and the signal returns here only
   // once we are scheduled back in, which is the strong round-trip.
   pthread_kill(pthread_self(), preempt_signo);
}

void cyros_port_pend_reschedule(void)
{
   // Weak-guarantee, deferred-safe. No thread context means nothing to switch
   // away from.
   if (!current_core.current_context) return;

   // Raise the reschedule signal against ourselves. At baseline it is delivered
   // before this returns. If a depth counter has it masked it stays pending and
   // is delivered the moment the matching enable unmasks it. The kernel pending
   // bit is the deferral mechanism, so no separate flag is required.
   pthread_kill(pthread_self(), preempt_signo);
}

void cyros_port_thread_exit(cyros_mask_token_t token)
{
   CYROS_ASSERT(current_core.preempt_disable_depth > 0); // thread_exit routine must be uninterruptible!
   CYROS_ASSERT(global.active_contexts.load(std::memory_order_relaxed) != 0);

   uint32_t remaining = global.active_contexts.fetch_sub(1, std::memory_order_seq_cst) - 1;

   // One idle thread per core remaining means the system has quiesced. The first
   // terminator to observe this wins the CAS and wakes every core to unwind.
   if (remaining <= global.cores.size()) {
      bool expected = false;
      if (global.shutdown_requested.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
         for (auto& core : global.cores) {
            pthread_kill(core.pthread, preempt_signo); // pends on us, blocked above
         }
      }
   }

   // The next switch must not write this dead context back into the terminated
   // TCB, whose joiner may already be tearing it down.
   current_core.discard_outgoing = true;

   // Pend our final reschedule, then unmask to let it fire. The handler discards
   // this context and resumes either the next thread or, under shutdown, the
   // bring-up context. We do not return.
   pthread_kill(pthread_self(), preempt_signo);
   cyros_port_preempt_enable(token);
   CYROS_PORT_UNREACHABLE(); // the enable above must not come back
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
 * ------------------------------------------------------------------------- */

void cyros_port_idle(void)
{
   // Park until a reschedule signal arrives. Delivery runs the handler, which
   // reschedules this core. If a thread became ready we are switched out and do
   // not return here until parked as idle again. sigsuspend returns once a
   // handler has run and we were resumed as the idle thread.
   sigset_t wait_mask;
   sigemptyset(&wait_mask); // wait with everything deliverable
   sigsuspend(&wait_mask);  // always returns -1 with EINTR after a handler runs
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

