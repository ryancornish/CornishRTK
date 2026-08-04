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

} // namespace cyros::port

#endif /* CYROS_PORT_LINUX_PREEMPT_INTERNAL_HPP */
