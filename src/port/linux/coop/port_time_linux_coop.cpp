/**
 * @file port_linux_coop.cpp
 * @brief Linux simulation time port using Boost.Context
 *
 * TODO: description
 */

#include <cyros/port/port_time.h>

#include <atomic>
#include <cstdint>


/* ============================================================================
 * Time Driver Port
 *
 * Provides monotonic time for real drivers (periodic / tickless) in unit
 * tests, plus tickless one-shot arming and ISR delivery when pumped.
 *
 * Note:
 * - The simulation time driver owns time and does NOT use this.
 * - Periodic driver unit tests call on_timer_isr() directly.
 * ========================================================================= */

struct time_state
{
   std::atomic<bool>        irq_enabled{false};
   std::atomic<uint64_t>            now{0};
   std::atomic<uint64_t> armed_deadline{UINT64_MAX};

   std::atomic<cyros_port_isr_handler_t> isr{nullptr};
   std::atomic<void*>                 isr_arg{nullptr};
};
static constinit time_state time_instance;

void cyros_port_time_setup(uint32_t tick_hz)
{
   (void)tick_hz;
}

uint64_t cyros_port_time_now(void)
{
   return time_instance.now.load(std::memory_order_relaxed);
}

uint64_t cyros_port_time_freq_hz(void)
{
   return 1'000'000ull; // 1 tick = 1 us (recommend)
}

void cyros_port_time_reset(uint64_t t)
{
   time_instance.now.store(t, std::memory_order_release);
   time_instance.armed_deadline.store(UINT64_MAX, std::memory_order_release);
}

void cyros_port_time_register_isr_handler(cyros_port_isr_handler_t h, void* arg)
{
   time_instance.isr_arg.store(arg, std::memory_order_relaxed);
   time_instance.isr.store(h, std::memory_order_release);
}

void cyros_port_time_irq_enable(void)  { time_instance.irq_enabled.store(true,  std::memory_order_release); }
void cyros_port_time_irq_disable(void) { time_instance.irq_enabled.store(false, std::memory_order_release); }

void cyros_port_time_arm(uint64_t deadline)
{
   // Keep earliest
   uint64_t cur = time_instance.armed_deadline.load(std::memory_order_relaxed);
   while (deadline < cur &&
            !time_instance.armed_deadline.compare_exchange_weak(cur, deadline,
                                                   std::memory_order_release,
                                                   std::memory_order_relaxed))
   {}
}

void cyros_port_time_disarm(void)
{
   time_instance.armed_deadline.store(UINT64_MAX, std::memory_order_release);
}

// Linux-only helper for tests
extern void cyros_port_time_advance(uint64_t delta)
{
   time_instance.now.fetch_add(delta, std::memory_order_release);
}

extern void cyros_port_time_fire_isr(void)
{
   auto handler = time_instance.isr.load(std::memory_order_acquire);
   if (handler) {
      handler(time_instance.isr_arg.load(std::memory_order_acquire));
   }
}

void cyros_port_send_time_ipi(uint32_t /*core_id*/)
{
   // SMP simulation TODO: poke target core thread.
}
