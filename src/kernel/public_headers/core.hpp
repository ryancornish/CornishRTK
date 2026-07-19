#ifndef CYROS_CORE_HPP
#define CYROS_CORE_HPP

#include <cstdint>

namespace cyros::this_core
{

/**
   * @brief Get current CPU core ID (0-based)
   */
[[nodiscard]] std::uint32_t id() noexcept;

/**
 * @brief Request a deferred reschedule on the calling core.
 *
 * Pends a reschedule on the current core, safe to call from an ISR.
 * @note This may return without rescheduling. If so, the reschedule is deferred and resolved
 * at the next safe point.
 */
void pend_reschedule() noexcept;

struct [[nodiscard]] preemption_token { std::uint32_t v; };

preemption_token disable_preemption() noexcept;

void enable_preemption(preemption_token token) noexcept;

struct [[nodiscard]] critical_token { std::uint32_t v; };

/**
 * @brief Enter an interrupt-masking critical section on the calling core (nestable).
 *
 * The stronger sibling of disable_preemption(): interrupts cannot be
 * delivered to this core until the matching exit_critical(), so the section
 * is atomic with respect to ISRs as well as thread switches.
 */
critical_token enter_critical() noexcept;

/**
 * @brief Leave an interrupt-masking critical section (nestable).
 *
 * When this restores the core to baseline (no masking at any grade), any
 * reschedule or interrupt that pended during the section is resolved before
 * this call returns.
 */
void exit_critical(critical_token token) noexcept;

struct preemption_guard
{
public:
   preemption_guard() noexcept
   {
      token = disable_preemption();
   }

   ~preemption_guard() noexcept
   {
      enable_preemption(token);
   }

   preemption_guard(preemption_guard const&)            = delete;
   preemption_guard& operator=(preemption_guard const&) = delete;
   preemption_guard(preemption_guard&&)                 = delete;
   preemption_guard& operator=(preemption_guard&&)      = delete;

private:
   preemption_token token{};
};

struct critical_guard
{
public:
   critical_guard() noexcept
   {
      token = enter_critical();
   }

   ~critical_guard() noexcept
   {
      exit_critical(token);
   }

   critical_guard(critical_guard const&)            = delete;
   critical_guard& operator=(critical_guard const&) = delete;
   critical_guard(critical_guard&&)                 = delete;
   critical_guard& operator=(critical_guard&&)      = delete;

private:
   critical_token token{};
};

} // namespace cyros::this_core

#endif // CYROS_CORE_HPP
