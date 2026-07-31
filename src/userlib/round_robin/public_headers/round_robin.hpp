/**
 * @file round_robin.hpp
 * @brief Round-robin time-slicing: rotate equal-priority threads on a timer.
 */
#ifndef CYROS_ROUND_ROBIN_HPP
#define CYROS_ROUND_ROBIN_HPP

#include <cyros/time/time.hpp>
#include <cyros/kernel/visibility.hpp>

namespace cyros::rr
{

/**
 * @brief Enable round-robin time-slicing on the calling core.
 *
 * Arms a recurring timer whose callback pends a reschedule, so every 'slice' the
 * running thread is re-enqueued behind its equal-priority peers and the next peer
 * runs. Threads of strictly higher priority are unaffected, since a rotation only
 * re-picks among the highest ready priority.
 *
 * Per-core: this slices the calling core only. To time-slice every core, call it
 * from each core's init. It may be called before or after that core's
 * time::start(). A call before start pends and begins at release, so slicing
 * across all cores can be set up during bring-up and released together.
 *
 * Runs until time is finalised. There is no separate disable, matching that
 * round-robin is a policy left on for the life of the system.
 */
CYROS_PUBLIC void enable_round_robin(time::duration slice);

} // namespace cyros::rr

#endif // CYROS_ROUND_ROBIN_HPP
