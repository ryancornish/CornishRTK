#ifndef CYROS_TIME_HPP
#define CYROS_TIME_HPP

#include <cstdint>
#include <limits>
#include <cyros/kernel/visibility.hpp>

namespace cyros::time
{

/* ============================================================================
 * Time Types
 * ========================================================================= */

/**
 * @brief Monotonic time point in driver ticks.
 *
 * A time_point is an absolute timestamp measured in the logical tick units of
 * the active time driver. The underlying unit is defined by the driver
 * frequency passed to initialise().
 *
 * Time points may be compared directly. Use duration_between() to form a
 * non-negative duration between two points.
 */
struct time_point
{
   uint64_t value{0};

   constexpr time_point() = default;
   constexpr explicit time_point(uint64_t v) : value(v) {}

   constexpr bool operator==(time_point rhs) const { return value == rhs.value; }
   constexpr bool operator!=(time_point rhs) const { return value != rhs.value; }
   constexpr bool operator< (time_point rhs) const { return value <  rhs.value; }
   constexpr bool operator<=(time_point rhs) const { return value <= rhs.value; }
   constexpr bool operator> (time_point rhs) const { return value >  rhs.value; }
   constexpr bool operator>=(time_point rhs) const { return value >= rhs.value; }

   [[nodiscard]] static constexpr time_point max()
   {
      return time_point{std::numeric_limits<uint64_t>::max()};
   }
};

/**
 * @brief Time duration in driver ticks.
 *
 * A duration is a relative span of time measured in the same logical units as
 * time_point.
 */
struct duration
{
   uint64_t value{0};

   constexpr duration() = default;
   constexpr explicit duration(uint64_t v) : value(v) {}

   constexpr time_point operator+(time_point tp) const
   {
      return time_point{tp.value + value};
   }

   constexpr duration operator+(duration rhs) const
   {
      return duration{value + rhs.value};
   }

   constexpr bool operator==(duration rhs) const { return value == rhs.value; }
   constexpr bool operator< (duration rhs) const { return value <  rhs.value; }
   constexpr bool operator> (duration rhs) const { return value >  rhs.value; }
};

/**
 * @brief Add a duration to a time point.
 */
constexpr time_point operator+(time_point tp, duration d)
{
   return time_point{tp.value + d.value};
}

/**
 * @brief Return the non-negative duration between two time points.
 *
 * If @p a is earlier than @p b, the result is clamped to zero.
 */
constexpr duration duration_between(time_point a, time_point b)
{
   return (a.value >= b.value) ? duration{a.value - b.value} : duration{0};
}

/**
 * @brief Scheduled callback function type.
 */
using callback = void(*)(void*);

/**
 * @brief Opaque handle for a scheduled callback.
 *
 * A handle with id == 0 is invalid.
 */
struct handle
{
   uint32_t id{0};
};


/* ============================================================================
 * Time Driver Interface - Must be implemented by a specific time driver
 * ========================================================================= */

/**
 * @brief Initialise the active time driver.
 *
 * @param frequency_hz Logical driver frequency in Hz.
 *
 * This frequency defines the tick units used by time_point, duration, and the
 * conversion helpers such as from_milliseconds().
 */
CYROS_PUBLIC void initialise(uint32_t frequency_hz);

/**
 * @brief Finalise the active time driver.
 *
 * Releases any driver-owned state and returns the time subsystem to an
 * uninitialised state.
 */
CYROS_PUBLIC void finalise();

/**
 * @brief Get the current monotonic time.
 *
 * @return Current time in driver ticks.
 */
[[nodiscard]] CYROS_PUBLIC time_point now() noexcept;

/**
 * @brief Schedule a callback to run at or after a specific time point.
 *
 * @param tp Absolute deadline in driver ticks.
 * @param cb callback to invoke.
 * @param arg User argument passed to the callback.
 * @return handle for later cancellation, or an invalid handle on failure.
 *
 * The callback may execute in interrupt context on embedded targets, or in the
 * caller / simulation context depending on the active driver.
 */
[[nodiscard]] CYROS_PUBLIC handle schedule_at(time_point tp, callback cb, void* arg) noexcept;

/**
 * @brief Schedule a callback to run repeatedly at a fixed interval.
 *
 * @param interval Period between invocations in driver ticks. Must be non-zero.
 * @param cb callback to invoke.
 * @param arg User argument passed to the callback.
 * @return handle for later cancellation, or an invalid handle on failure.
 *
 * The first invocation occurs one interval from now, and every interval
 * thereafter. Unlike schedule_at(), a recurring callback never stops on its
 * own, so the returned handle must be passed to cancel() to stop it.
 *
 * The callback may execute in interrupt context on embedded targets, or in the
 * caller / simulation context depending on the active driver.
 */
[[nodiscard]] CYROS_PUBLIC handle schedule_recurring(duration interval, callback cb, void* arg) noexcept;

/**
 * @brief Cancel a scheduled callback.
 *
 * @param h handle returned by schedule_at().
 * @return True if the callback was cancelled before firing, false otherwise.
 *
 * It is safe to cancel an invalid, unknown, or already-fired handle.
 */
CYROS_PUBLIC bool cancel(handle h) noexcept;

/**
 * @brief Convert milliseconds to a driver duration.
 *
 * Conversion is rounded up so that non-zero durations do not undersleep.
 */
[[nodiscard]] CYROS_PUBLIC duration from_milliseconds(uint32_t ms) noexcept;

/**
 * @brief Convert microseconds to a driver duration.
 *
 * Conversion is rounded up so that non-zero durations do not undersleep.
 */
[[nodiscard]] CYROS_PUBLIC duration from_microseconds(uint32_t us) noexcept;

/**
 * @brief Convert a driver duration to milliseconds.
 *
 * Conversion is rounded to the nearest millisecond.
 */
[[nodiscard]] CYROS_PUBLIC uint64_t to_milliseconds(duration d) noexcept;

/**
 * @brief Convert a driver duration to microseconds.
 *
 * Conversion is rounded to the nearest microsecond.
 */
[[nodiscard]] CYROS_PUBLIC uint64_t to_microseconds(duration d) noexcept;

/**
 * @brief Start the time driver.
 *
 * Enables time progression and any required timer interrupt or background
 * mechanism for the active driver.
 */
CYROS_PUBLIC void start() noexcept;

/**
 * @brief Stop the time driver.
 *
 * Disables time progression mechanisms used by the active driver.
 */
CYROS_PUBLIC void stop() noexcept;

} // namespace cyros::time

#endif // CYROS_TIME_HPP
