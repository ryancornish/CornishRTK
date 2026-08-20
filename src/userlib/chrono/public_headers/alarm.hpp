#ifndef CYROS_CHRONO_ALARM_HPP
#define CYROS_CHRONO_ALARM_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/visibility.hpp>
#include <cyros/time/time.hpp>

#include <atomic>

namespace cyros::chrono
{

/**
 * @brief A one-shot waitable deadline.
 *
 * Wraps a one-shot time:: timer whose expiry satisfies every waiter. Expiry is
 * a level, not an event to consume: once fired, any thread that waits on this
 * alarm is satisfied until it is re-armed. That is what makes a fire that lands
 * before the waiter parks harmless, the pre-park poll sees it.
 *
 * Composes with the group wait: wait_on_any(thing, alarm) is a timed wait on
 * the thing, and which index comes back says whether it was the deadline.
 *
 * @note Arm and destroy on the same thread. The timer callback runs on the
 *       arming core, so a same-thread disarm or destroy cannot race it. There
 *       is deliberately no support for destroying an armed alarm from another
 *       thread.
 */
class CYROS_PUBLIC alarm : public waitable
{
public:
   alarm() noexcept = default;

   /**
    * @brief Destroying an armed alarm disarms it first.
    */
   ~alarm() override;

   /**
    * @brief Arm to fire at @p tp. The alarm must not already be pending.
    *
    * A deadline at or before now fires on the driver's next evaluation, which
    * is the schedule_at contract, not something alarm papers over.
    */
   void arm_at(time::time_point tp) noexcept;

   /**
    * @brief Arm to fire @p d from now. The alarm must not already be pending.
    */
   void arm_in(time::duration d) noexcept;

   /**
    * @brief Return to idle: cancel a pending timer, clear an expired one.
    *
    * Safe in every state. If the timer is mid-fire when the cancel misses
    * (reachable only under the simulation driver, whose pump runs callbacks
    * outside its lock), this waits out the two-statement callback so that
    * returning from disarm always means the callback is finished with this
    * object.
    */
   void disarm() noexcept;

   /**
    * @brief True once the deadline has fired and the alarm has not been
    *        re-armed or disarmed since.
    */
   [[nodiscard]] bool expired() const noexcept;

protected:
   /// Non-consuming: a fired alarm satisfies every waiter that asks.
   bool try_satisfy() noexcept override;

private:
   static void on_timer(void* self) noexcept;

   [[nodiscard]] bool pending() const noexcept;

   std::atomic<bool> fired{false};
   time::handle timer{};
};

} // namespace cyros::chrono

#endif // CYROS_CHRONO_ALARM_HPP
