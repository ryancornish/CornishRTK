#include <cyros/kernel/spinlock.hpp>
#include <cyros/port/port.h>

namespace cyros
{

void spinlock::lock()
{
   // Interrupt-masking grade, deliberately. A preemption-only grade would let
   // an ISR interrupt a holder on its own core, and an ISR wake path that
   // then took this lock would spin against its own interrupted thread
   // forever. Masking first also means contention spins with interrupts
   // masked, which is the conventional trade (holders release in bounded
   // tiny time by contract) and what keeps the acquire race ISR-free.
   token = this_core::enter_critical();
   while (flag.test_and_set(std::memory_order_acquire)) {
      // busy-wait with cpu yield hint
      cyros_port_cpu_relax();
   }
}

void spinlock::unlock()
{
   flag.clear(std::memory_order_release);
   this_core::exit_critical(token);
}

} // namespace cyros
