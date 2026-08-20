#include <cyros/kernel/core.hpp>
#include <cyros/port/port.h>

namespace cyros::this_core
{

[[nodiscard]] std::uint32_t id() noexcept
{
   return cyros_port_get_core_id();
}

void pend_reschedule() noexcept
{
   cyros_port_pend_reschedule();
}

preemption_token disable_preemption() noexcept
{
   return { .v = cyros_port_preempt_disable() };
}

void enable_preemption(preemption_token token) noexcept
{
   cyros_port_preempt_enable(token.v);
}

critical_token enter_critical() noexcept
{
   return { .v = cyros_port_irq_save() };
}

void exit_critical(critical_token token) noexcept
{
   cyros_port_irq_restore(token.v);
}

} // namespace cyros::this_core
