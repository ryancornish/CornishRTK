/**
 * @file port_linux_preempt_internal.hpp
 * @brief Contract between port_linux_preempt.cpp and its time driver.
 *
 * Port-internal only. Nothing here is part of the kernel-facing port API and no
 * other component may include it. Definitions live in port_linux_preempt.cpp
 * because they touch its thread-local core state, which stays private there.
 */

#ifndef CYROS_PORT_LINUX_PREEMPT_INTERNAL_HPP
#define CYROS_PORT_LINUX_PREEMPT_INTERNAL_HPP

namespace cyros::port
{

/**
 * @brief Declare that an interrupt handler is running on this core.
 *
 * The port derives the OS signal mask from its depth counters, so every phase
 * that masks an owned signal must be expressed as a depth. A signal handler is
 * such a phase: delivery blocks the signal plus sa_mask before any of our code
 * runs. Without this, a cyros_port_irq_restore() inside a handler would reopen a
 * signal the handler still needs held down. Rationale in
 * port_linux_preempt.cpp under "Interrupt-handler regions".
 *
 * Prefer isr_region wherever scoping allows.
 */
void isr_region_enter();
void isr_region_leave();

/** @brief Scoped form of isr_region_enter/leave. */
struct isr_region
{
   isr_region() { isr_region_enter(); }
   ~isr_region() { isr_region_leave(); }

   isr_region(isr_region const&)            = delete;
   isr_region(isr_region&&)                 = delete;
   isr_region& operator=(isr_region const&) = delete;
   isr_region& operator=(isr_region&&)      = delete;
};

/**
 * @brief Declare that the reschedule interception is running on this core.
 *
 * Preempt-disabled, not interrupt-disabled: only the trigger signal is held
 * down, so a timer can preempt the handler. Load-bearing rather than
 * bookkeeping, because a preempt_enable inside the handler recomputes the mask
 * from the counters, and without this it would unblock the trigger and let the
 * interception re-enter itself on the shared handler stack.
 *
 * Prefer preempt_region wherever scoping allows.
 */
void preempt_region_enter();
void preempt_region_leave();

/** @brief Scoped form of preempt_region_enter/leave. */
struct preempt_region
{
   preempt_region() { preempt_region_enter(); }
   ~preempt_region() { preempt_region_leave(); }

   preempt_region(preempt_region const&)            = delete;
   preempt_region(preempt_region&&)                 = delete;
   preempt_region& operator=(preempt_region const&) = delete;
   preempt_region& operator=(preempt_region&&)      = delete;
};

} // namespace cyros::port

#endif /* CYROS_PORT_LINUX_PREEMPT_INTERNAL_HPP */
