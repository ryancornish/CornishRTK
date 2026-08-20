#ifndef CYROS_CHRONO_CHRONO_HPP
#define CYROS_CHRONO_CHRONO_HPP

#include <cyros/kernel/visibility.hpp>
#include <cyros/time/time.hpp>

/**
 * @file chrono.hpp
 * @brief The chrono feature's vocabulary and sleeping.
 *
 * chrono re-exports the time driver's vocabulary so a user browsing this
 * feature finds time_point and duration where std habits expect them. The
 * types ARE the time:: types, these are aliases, not wrappers.
 */

namespace cyros::chrono
{

using time_point = time::time_point;
using duration   = time::duration;

using time::now;
using time::from_milliseconds;
using time::from_microseconds;

/**
 * @brief Block the calling thread until @p tp.
 *
 * Wakes at or after the deadline, on the driver's granularity. A deadline
 * already in the past returns on the driver's next evaluation.
 */
CYROS_PUBLIC void sleep_until(time::time_point tp) noexcept;

/**
 * @brief Block the calling thread for @p d.
 *
 * A zero duration returns immediately without yielding. A yield is spelled
 * this_thread::yield().
 */
CYROS_PUBLIC void sleep_for(time::duration d) noexcept;

} // namespace cyros::chrono

namespace cyros::this_thread
{
// The std pattern: sleeping is a this_thread activity.
using cyros::chrono::sleep_for;
using cyros::chrono::sleep_until;
} // namespace cyros::this_thread

#endif // CYROS_CHRONO_CHRONO_HPP
