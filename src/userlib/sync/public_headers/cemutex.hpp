#ifndef CYROS_CEMUTEX_HPP
#define CYROS_CEMUTEX_HPP

#include <cyros/kernel/base_mutex.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/visibility.hpp>

namespace cyros::sync
{

/**
 * @brief Mutual exclusion with an immediate priority ceiling.
 *
 * A sibling of sync::mutex over the same kernel base_mutex, differing only in
 * what a held lock contributes to its holder's urgency: a constant here, the
 * best waiter there. A separate type rather than a runtime flag so that which
 * protocol a lock runs is visible where the lock is declared, which is the one
 * thing a reader most wants to know about it.
 *
 * The holder runs at the ceiling from acquisition until release, whether or not
 * anyone is waiting. That is the difference in a sentence: inheritance repairs
 * an inversion once it happens, a ceiling prevents it from happening. The cost
 * is that a lightly contended lock pays the boost every time, which is why both
 * types exist rather than one.
 *
 * @param ceiling Must be at least as urgent (numerically <=) as the base
 *        priority of EVERY thread that can lock this. Asserted on each acquire.
 *        Getting it wrong is not a tuning error, it reinstates the inversion.
 *
 * @note Deadlock immunity is NOT claimed. The classic ceiling result assumes a
 *       single processor. Cyros pins threads to cores and schedules them
 *       independently, so a thread on another core can and does block on a
 *       ceiling lock, and two locks taken in opposite orders still deadlock.
 *       What the ceiling buys here is bounded blocking and no inversion, not
 *       the uniprocessor theorem.
 *
 * @see mutex-first-class-plan.md D6
 */
class CYROS_PUBLIC cemutex : public base_mutex
{
public:
   constexpr explicit cemutex(thread::priority ceiling) noexcept
      : base_mutex(static_cast<std::uint8_t>(ceiling))
   {}
};

}  // namespace cyros::sync

namespace cyros { using sync::cemutex; }

#endif // CYROS_CEMUTEX_HPP
