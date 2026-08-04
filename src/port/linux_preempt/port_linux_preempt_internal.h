/**
 * @file port_linux_preempt_internal.h
 * @brief Contract between port_linux_preempt.cpp and its time driver.
 *
 * Port-internal only. Nothing here is part of the kernel-facing port API and no
 * other component may include it. Definitions live in port_linux_preempt.cpp
 * because they touch its thread-local core state, which stays private there.
 */

#ifndef CYROS_PORT_LINUX_PREEMPT_INTERNAL_H
#define CYROS_PORT_LINUX_PREEMPT_INTERNAL_H

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
 * Prefer cyros_port_isr_region wherever scoping allows.
 */
void cyros_port_isr_region_enter();
void cyros_port_isr_region_leave();

/** @brief Scoped form of cyros_port_isr_region_enter/leave. */
struct cyros_port_isr_region
{
   cyros_port_isr_region() { cyros_port_isr_region_enter(); }
   ~cyros_port_isr_region() { cyros_port_isr_region_leave(); }

   cyros_port_isr_region(cyros_port_isr_region const&)            = delete;
   cyros_port_isr_region(cyros_port_isr_region&&)                 = delete;
   cyros_port_isr_region& operator=(cyros_port_isr_region const&) = delete;
   cyros_port_isr_region& operator=(cyros_port_isr_region&&)      = delete;
};

#endif /* CYROS_PORT_LINUX_PREEMPT_INTERNAL_H */
