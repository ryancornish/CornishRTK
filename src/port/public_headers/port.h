/**
 * @file port.h
 * @brief Cyros Port Layer API (C ABI)
 *
 * This is the main* contract between the kernel component, and the hardware/platform.
 * Code that implements this contract's API for a specific platforms architecture is
 * a 'port implementation'. A port implementation must implemented definitions for ALL
 * functions declared here.
 *
 * The API uses C linkage for a _simpler_ ABI. This allows port implemenetations to
 * be in C or assembly. Though C++ is still available.
 *
 * *This is the _main_ contract, but not the only port contract. More specifically,
 * this is the contract between the kernel component of Cyros and the platform.
 * It is the bare-minimum implementable for a complete Cyros build. Other components
 * such as the time-driver also require a port implementation. If the Cyros build requires
 * a time-driver, then it also requires the port component to implement it. The contract
 * for the time driver is separate-but-parallel to this contract (see port_time.h).
 */

#ifndef CYROS_PORT_H
#define CYROS_PORT_H

#include <cyros/port/port_traits.h>

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Port Configuration Validation
 * ----------------------------------------------------------------------------
 * Verify and validate the port_traits.h implementation exposes all
 * necessary macro definitions and that the values are sane.
 * ========================================================================= */

#ifndef CYROS_PORT_CONTEXT_SIZE
# error "Port must define CYROS_PORT_CONTEXT_SIZE"
#endif

#ifndef CYROS_PORT_CONTEXT_ALIGN
# error "Port must define CYROS_PORT_CONTEXT_ALIGN"
#endif

#ifndef CYROS_PORT_STACK_ALIGN
# error "Port must define CYROS_PORT_STACK_ALIGN"
#endif

#ifndef CYROS_PORT_CACHE_LINE
# error "Port must define CYROS_PORT_CACHE_LINE"
#endif

#ifndef CYROS_PORT_CORE_COUNT
# error "Port must define CYROS_PORT_CORE_COUNT"
#endif

#ifndef CYROS_PORT_CAPTURE_LOCATION
# error "Port must define CYROS_PORT_CAPTURE_LOCATION"
#endif

#ifndef CYROS_PORT_DEBUG_MODE
# error "Port must define CYROS_PORT_DEBUG_MODE"
#endif

#if (CYROS_PORT_CONTEXT_SIZE) <= 0
# error "CYROS_PORT_CONTEXT_SIZE must be > 0"
#endif

#if (CYROS_PORT_CONTEXT_ALIGN) <= 0
# error "CYROS_PORT_CONTEXT_ALIGN must be > 0"
#endif

#if (CYROS_PORT_STACK_ALIGN) <= 0
# error "CYROS_PORT_STACK_ALIGN must be > 0"
#endif

#if (CYROS_PORT_CACHE_LINE) <= 0
# error "CYROS_PORT_CACHE_LINE must be > 0"
#endif

#if (CYROS_PORT_CORE_COUNT) <= 0
# error "CYROS_PORT_CORE_COUNT must be > 0"
#endif

#if ((CYROS_PORT_CONTEXT_ALIGN & (CYROS_PORT_CONTEXT_ALIGN - 1)) != 0)
# error "CYROS_PORT_CONTEXT_ALIGN must be a power of two"
#endif

#if ((CYROS_PORT_STACK_ALIGN & (CYROS_PORT_STACK_ALIGN - 1)) != 0)
# error "CYROS_PORT_STACK_ALIGN must be a power of two"
#endif

#if ((CYROS_PORT_CACHE_LINE & (CYROS_PORT_CACHE_LINE - 1)) != 0)
# error "CYROS_PORT_CACHE_LINE must be a power of two"
#endif

#if (CYROS_PORT_CAPTURE_LOCATION != 0) &&  (CYROS_PORT_CAPTURE_LOCATION != 1)
# error "CYROS_PORT_CAPTURE_LOCATION must be '0' (false) or '1' (true)"
#endif

#if (CYROS_PORT_DEBUG_MODE != 0) &&  (CYROS_PORT_DEBUG_MODE != 1)
# error "CYROS_PORT_DEBUG_MODE must be '0' (false) or '1' (true)"
#endif


/* ============================================================================
 * Port Type Definitions
 * ----------------------------------------------------------------------------
 * Various types to support the contract API.
 * ========================================================================= */

/**
 * @brief Opaque context structure (platform-specific size/alignment)
 *
 * Each port defines the actual structure. The kernel treats this as opaque.
 */
typedef struct cyros_port_context cyros_port_context_t;

/**
 * @brief Port->Kernel reschedule hook
 *
 * Installed by the kernel via cyros_port_init(). The port invokes this hook
 * when a reschedule must be serviced on the calling core - i.e. as the
 * back-end of both cyros_port_thread_yield() and cyros_port_pend_reschedule()
 * once the port has determined a reschedule may safely run. The hook runs the
 * core-local scheduler, which may perform a context switch.
 */
typedef void (*cyros_port_reschedule_t)(void);

/**
 * @brief Thread entry point signature
 *
 * MUST NOT RETURN.
 * A port is free to enforce this however it likes, or not at all.
 */
typedef void (*cyros_port_entry_t)(void* arg);

/**
 * @brief Core entry point signature
 */
typedef void (*cyros_port_core_entry_t)(void);

/**
 * @brief Opaque token to represent a nested interrupt state
 *
 * Its bits are assigned by the active port and are
 * meaningful only to that port's matching enable/restore. Obtain one from a
 * *_disable() / *_save() call and hand it back to the paired enable/restore,
 * and nowhere else.
 */
typedef uint32_t cyros_mask_token_t;


/* ============================================================================
 * Port Contract API
 * ----------------------------------------------------------------------------
 * Functions that must be implemented by a port implementation.
 * Sections include:
 * - Platform Initialisation
 * - SMP & Multi-Core Support
 * - Interrupt Control
 * - Preemption Control
 * - Context Management & Switching
 * - Reschedule Requests
 * - Thread-Local Storage
 * - CPU Hints & Idle
 * - Debug & Diagnostics
 * ========================================================================= */

/* ----------------------------------------------------------------------------
 * Platform Initialisation
 * ------------------------------------------------------------------------- */

/**
 * @brief Initialise the port layer
 * @param reschedule_handler Port->kernel hook invoked to service a reschedule
 *                           on the calling core (see cyros_port_reschedule_t).
 */
void cyros_port_init(cyros_port_reschedule_t reschedule_handler);


/* ----------------------------------------------------------------------------
 * SMP & Multi-Core Support
 *
 * Each pthread represents a simulated "core". Core 0 runs on the calling
 * thread, additional cores spawn as pthreads.
 * ------------------------------------------------------------------------- */

/**
 * @brief Get the ID of the current CPU core
 * @return Core ID (0-indexed)
 *
 * For single-core systems, always returns 0.
 * For SMP systems, returns which core is executing this code.
 */
uint32_t cyros_port_get_core_id(void);

/**
 * @brief Start (or release) all secondary cores and run entry on every core.
 * @param cores_to_use Number of cores to start
 * @param entry Entry point to run on each core
 *
 * After this call returns on the bootstrap core:
 *  - On embedded: typically never returns because entry will start the first thread.
 *  - On simulation: may return if port_start_first returns (cooperative).
 */
void cyros_port_start_cores(size_t cores_to_use, cyros_port_core_entry_t entry);

/**
 * @brief Send an IPI to another core to trigger a reschedule
 * @param core_id Target core ID
 *
 * Causes the target core to perform a reschedule at its next safe point. This
 * is the cross-core analogue of cyros_port_pend_reschedule(): it carries the
 * same weak guarantee and the receiving core resolves it exactly as a locally
 * pended reschedule would be.
 */
void cyros_port_send_reschedule_ipi(uint32_t core_id);


/* ----------------------------------------------------------------------------
 * Interrupt Control
 *
 * Interrupt masking blocks the *hardware*: while raised, asynchronous
 * interrupts cannot be delivered to the calling core. This is distinct from
 * preemption control (below) - see the note in the Preemption Control section.
 * ------------------------------------------------------------------------- */

/**
 * @brief Check if interrupts are currently enabled
 * @return true if interrupts are enabled, false otherwise
 */
bool cyros_port_interrupts_enabled(void);

/**
 * @brief Save interrupt state and disable interrupts
 * @return Previous interrupt state
 */
cyros_mask_token_t cyros_port_irq_save(void);

/**
 * @brief Restore interrupt state
 * @param token Previous state returned by cyros_port_irq_save()
 *
 * When this restores interrupts to fully unmasked AND preemption is also
 * enabled, the calling core is at baseline priority; if a reschedule was
 * pended while masked it is resolved at that point (see Reschedule Requests).
 */
void cyros_port_irq_restore(cyros_mask_token_t token);


/* ----------------------------------------------------------------------------
 * Preemption Control
 *
 * Preemption disabling blocks the *scheduler*: while raised, no context switch
 * may occur on the calling core. Interrupts are NOT affected - ISRs still fire
 * and run. They simply cannot cause a thread switch until preemption is
 * re-enabled.
 *
 * Preemption vs interrupt masking
 * -------------------------------
 * These are independent, separately-nesting facilities:
 *  - Interrupt masking defends against a device ISR running mid-mutation.
 *  - Preemption disabling defends against a thread switch while the caller
 *    holds something (e.g. a spinlock) that makes switching unsafe, while
 *    still permitting ISRs to run.
 * A context switch on a core can occur only when BOTH are at zero depth:
 * interrupts unmasked and preemption enabled. That joint condition is what
 * port.h refers to as "baseline priority".
 *
 * The port owns the preemption mechanism because it is platform-specific
 * (a real priority/mask action on bare metal, a counter on a cooperative
 * simulation port). The kernel owns the policy: it decides when to call
 * these, and with what nesting.
 *
 * Both functions nest. Calls must be balanced.
 * ------------------------------------------------------------------------- */

/**
 * @brief Disable preemption on the calling core (nestable).
 *
 * Increments the preemption-disable nesting depth. While the depth is non-zero
 * no context switch will be performed on this core. ISRs continue to run. A
 * reschedule they request via cyros_port_pend_reschedule() is recorded and
 * deferred.
 */
cyros_mask_token_t cyros_port_preempt_disable(void);

/**
 * @brief Enable preemption on the calling core (nestable).
 *
 * Decrements the preemption-disable nesting depth. When the depth reaches zero
 * AND interrupts are not masked, the calling core is at baseline priority:
 * this is a contract "safe point", and if a reschedule was pended while
 * preemption was disabled it is resolved here before this call returns.
 *
 * Must be balanced against cyros_port_preempt_disable().
 */
void cyros_port_preempt_enable(cyros_mask_token_t token);


/* ----------------------------------------------------------------------------
 * Context Management & Switching
 * ------------------------------------------------------------------------- */

/**
 * @brief Initialize a thread context
 * @param context Pointer to context structure (pre-allocated by kernel)
 * @param stack_base Pointer to the base (lowest address) of the stack
 * @param stack_size Size of the stack in bytes
 * @param entry Thread entry point function
 * @param arg Argument to pass to entry function
 *
 * This function sets up the context so that when port_switch() is called
 * with this context, the thread starts executing at entry(arg).
 */
void cyros_port_context_init(cyros_port_context_t* context,
                              void* stack_base,
                              size_t stack_size,
                              cyros_port_entry_t entry,
                              void* arg);

/**
 * @brief Destroy a thread context
 * @param context The dead thread's context. Never entered or resumed again.
 *
 * Called when a thread exits: by reschedule() when it terminates the current
 * thread, after the kernel bookkeeping and before the switch away from it.
 *
 * Must return normally to the outer reschedule().
 *
 * The port can use this to perform internal bookkeeping on context tracking, to
 * invalidate the context, or nothing at all if it has nothing to release. The
 * storage itself belongs to the caller and outlives this call, so a port must
 * not assume it may reuse or free it.
 *
 * The context may still be SUSPENDED when this is called, and on a port with
 * stackful contexts it usually is: the thread is parked inside the final
 * reschedule that retired it and is never resumed to unwind. Abandoning a
 * suspended context is the contract. Unwinding one is not required and must not
 * be attempted.
 */
void cyros_port_context_destroy(cyros_port_context_t* context);

/**
 * @brief Switch from one context to another
 * @param from Context to save, or NULL when there is nothing to save
 * @param to Context to restore and resume
 *
 * Saves the current CPU state into 'from' and loads the state from 'to'.
 * Execution resumes in 'to' context.
 *
 * 'from' is NULL in two cases, and they want the same handling: the first
 * switch on a core, where no context has run yet, and a switch away from a
 * thread that has just been retired, whose context must not be written back
 * because its joiner may already be tearing the TCB down.
 */
void cyros_port_switch(cyros_port_context_t* from, cyros_port_context_t* to);

/**
 * @brief Start executing the first thread
 * @param first First thread context to run
 *
 * This is called once at scheduler startup to begin execution.
 * Unlike port_switch(), there's no "from" context to save.
 */
void cyros_port_start_first(cyros_port_context_t* first);


/* ----------------------------------------------------------------------------
 * Reschedule Requests
 *
 * Cyros distinguishes TWO reschedule operations. They differ in their
 * preconditions and in the strength of their guarantee. The kernel selects the
 * correct one per call site; a port MUST implement both to this contract.
 *
 * Terminology
 * -----------
 *  - "thread context": executing inside a thread's context, not inside an ISR.
 *  - "kernel-masked": the kernel currently holds a critical section on this
 *    core - interrupts masked via cyros_port_irq_save() AND/OR preemption
 *    disabled via cyros_port_preempt_disable().
 *  - "baseline priority": thread context, with interrupts NOT masked AND
 *    preemption NOT disabled - i.e. both nesting depths at zero. On Cortex-M
 *    this is Thread mode with PRIMASK clear, BASEPRI not raised above the
 *    PendSV priority, and PendSV not otherwise gated.
 *  - "next safe point": the next moment, on this core, at which execution
 *    returns to baseline priority - typically the outermost
 *    cyros_port_irq_restore() or cyros_port_preempt_enable(), or ISR return.
 *
 * Why two operations
 * ------------------
 * A reschedule cannot always be performed at the instant it becomes
 * desirable: the requester may be inside an ISR, or inside a kernel critical
 * section, and in either case switching context immediately would be unsound.
 * Splitting the operation makes the caller's intent - and, importantly, the
 * caller's obligations - explicit at every call site.
 *
 * Summary
 * -------
 *  cyros_port_thread_yield()      strong guarantee, thread context only,
 *                                  baseline priority only, synchronous.
 *  cyros_port_pend_reschedule()   weak guarantee, callable from any context,
 *                                  may be deferred.
 * ------------------------------------------------------------------------- */

/**
 * @brief Yield the calling thread to the scheduler (synchronous reschedule).
 *
 * Strong-guarantee reschedule. The scheduler runs immediately on the calling
 * core; if another thread is selected, the caller is switched out and this
 * call does NOT return until the caller is later switched back in. When it
 * does return, a full reschedule round-trip has provably completed.
 *
 * Precondition - the caller MUST satisfy ALL of:
 *  - Called from thread context (NEVER from an ISR).
 *  - Called at baseline priority: interrupts not masked AND preemption not
 *    disabled.
 *
 * Ports that can observe the execution priority SHOULD assert the baseline
 * precondition inside this function.
 *
 * Use when the caller is deliberately giving up the CPU and depends on the
 * round-trip having completed on return - e.g. blocking in a wait operation,
 * an explicit yield, or a join. Callers MAY rely on post-switch state being
 * valid immediately after this function returns.
 *
 * Port implementation notes:
 *  - Cortex-M: pend PendSV, then DSB/ISB. From baseline priority PendSV is
 *    taken before the next instruction, making the call synchronous.
 *  - RISC-V: trigger the software interrupt (or switch directly); from
 *    baseline priority it is serviced immediately.
 *  - Linux/boost.context: resume the scheduler fiber.
 */
void cyros_port_thread_yield(void);

/**
 * @brief Request a reschedule on the calling core (deferred-safe).
 *
 * Weak-guarantee reschedule. Guarantees ONLY that a reschedule will occur on
 * this core at or before the next safe point. This call MAY return before any
 * context switch has happened.
 *
 * Callable from ANY context: thread context or ISR, with or without a kernel
 * critical section held.
 *
 * Behaviour by context:
 *  - Baseline priority: the next safe point is "now", so the reschedule
 *    resolves before this call returns. This is observationally identical to
 *    cyros_port_thread_yield() - but see the caller obligation below.
 *  - Kernel-masked, or ISR context: the request is recorded and the actual
 *    reschedule is deferred to the next safe point - the outermost
 *    cyros_port_irq_restore() or cyros_port_preempt_enable(), or ISR return.
 *
 * Caller obligation: callers MUST NOT assume that a context switch has
 * occurred, or that any post-switch state is valid, after this call returns.
 * This holds even in the baseline-priority case: the weak guarantee IS the
 * contract, independent of how a particular context happens to satisfy it. A
 * caller that needs the round-trip must use cyros_port_thread_yield().
 *
 * Use when the caller wants to flag that a reschedule may now be warranted but
 * is not itself blocking - e.g. after making a higher-priority thread ready,
 * from a signal/wake path, or from an ISR handing work to a thread.
 *
 * Port implementation notes:
 *  - Cortex-M: set the PendSV pending bit (ICSR.PENDSVSET). The hardware
 *    defers delivery until execution priority drops to the PendSV level.
 *  - RISC-V: set the machine software interrupt pending bit (CLINT MSIP). The
 *    hardware defers delivery until software interrupts are unmasked.
 *  - Linux/boost.context: if at baseline priority, resume the scheduler fiber
 *    now; otherwise set a per-core "reschedule pending" flag that the
 *    irq_restore / preempt_enable safe points (and simulated-ISR return)
 *    check and resolve.
 */
void cyros_port_pend_reschedule(void);


/* ----------------------------------------------------------------------------
 * Thread-Local Storage
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the TLS pointer for the current thread
 * @param tls_base Pointer to the thread's TLS block
 */
void cyros_port_set_tls_pointer(void* tls_base);

/**
 * @brief Get the current TLS pointer
 * @return Current thread's TLS base pointer
 */
void* cyros_port_get_tls_pointer(void);


/* ----------------------------------------------------------------------------
 * Atomics
 *
 * The kernel uses std::atomic and requires every operation to be atomic with
 * respect to ALL cores, not merely against interrupts on the calling one. A port
 * whose CPU has no lock-free read-modify-write must satisfy that by supplying its
 * own __atomic_* definitions; there is no entry point here for it.
 * ------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------
 * CPU Hints & Idle
 * ------------------------------------------------------------------------- */

/**
 * @brief CPU yield hint for busy-wait loops
 */
void cyros_port_cpu_relax(void);

/**
 * @brief Platform-specific idle behavior
 *
 * Called by the kernel's idle thread when no other threads are ready.
 * Can implement power-saving features or cooperative yielding.
 */
void cyros_port_idle(void);


/* ----------------------------------------------------------------------------
 * Debug & Diagnostics
 * ------------------------------------------------------------------------- */

/**
 * @brief Internal Kernel asserts
 * The Kernel has been setup incorrectly, or has hit an internal system error
 */
void cyros_port_system_error(uintptr_t auxilary1, uintptr_t auxilary2, char const* file_optional, int line_optional) __attribute__((noreturn));

#if CYROS_PORT_CAPTURE_LOCATION
 #define CYROS_PORT_CAPTURE_FILE (__FILE__)
 #define CYROS_PORT_CAPTURE_LINE (__LINE__)
#else
 #define CYROS_PORT_CAPTURE_FILE ""
 #define CYROS_PORT_CAPTURE_LINE 0
#endif

#define CYROS_ASSERT2(condition, aux1, aux2) __builtin_expect(!!(condition), 1) ? (void)0 : cyros_port_system_error((uintptr_t)(aux1), (uintptr_t)(aux2), CYROS_PORT_CAPTURE_FILE, CYROS_PORT_CAPTURE_LINE)
#define CYROS_ASSERT1(condition, aux1)       CYROS_ASSERT2(condition, aux1, 0)
#define CYROS_ASSERT(condition)              CYROS_ASSERT2(condition, 0, 0)
#define CYROS_ASSERT_OP(lhs, op, rhs)        CYROS_ASSERT2((lhs) op (rhs), lhs, rhs)
#define CYROS_ASSERT_NULL(pointer)           CYROS_ASSERT2(!(pointer), pointer, 0);

/**
 * @def CYROS_PORT_UNREACHABLE
 * @brief Mark a path the kernel guarantees is never taken.
 */
#if CYROS_PORT_DEBUG_MODE
 #define CYROS_PORT_UNREACHABLE() cyros_port_system_error(0, 0, CYROS_PORT_CAPTURE_FILE, CYROS_PORT_CAPTURE_LINE)
#else
 #define CYROS_PORT_UNREACHABLE() __builtin_trap()
#endif

/**
 * @brief hold the current core until GDB is attached
 *
 * This may be implemented as a spin-loop, and resumption may
 * require GDB commands (e.g. resetting the value of a local)
 */
void cyros_port_wait_for_debugger(void);

/**
 * @brief Trigger a breakpoint (for debugging)
 */
void cyros_port_breakpoint(void);

/**
 * @brief Get the current stack pointer value
 * @return Current stack pointer
 */
void* cyros_port_get_stack_pointer(void);


#ifdef __cplusplus
}
#endif

#endif /* CYROS_PORT_H */
