#ifndef CYROS_THREAD_ACTION_HPP
#define CYROS_THREAD_ACTION_HPP

#include "scheduler.hpp"
#include "threading_subsystem.hpp"

namespace cyros::thread_action
{

/**
 * @brief Retrieve the currently running threads TCB on the calling core
 */
[[nodiscard, gnu::pure]]
thread_control_block& get_current_thread_on_this_core();

void register_thread(thread_control_block& tcb);

/**
 * @brief Make a thread runnable on its pinned core (smp-safe).
 *
 * transitions @p tcb to the ready state and enqueues it on the ready queue
 * of its pinned core. if the thread belongs to another core, a cross-core
 * request is posted so that the owning scheduler performs the enqueue.
 *
 * @param tcb thread control block of the thread to make runnable.
 * @return schedule_hint::warranted if the thread was queued on the current
 *         core and has higher priority than the running thread, indicating
 *         the caller should request a local reschedule.
 */
schedule_hint ready_thread(thread_control_block& tcb);

/**
 * @brief A thread's urgency, computed from truth: min(base, best waiter of
 *        every pi_waitable it holds).
 *
 * This is the DEFINITION of priority inheritance, evaluated rather than read.
 * There is no stored effective priority and no walk maintaining one: a cache
 * whose inputs live on other cores is what every priority-inheritance bug in
 * this project's history has been.
 *
 * Callable by ANY core with no lock held. That is not incidental: held_slots is
 * a bounded array of atomics rather than an intrusive list precisely so a
 * remote core can read what a thread holds without taking its pi_lock, and
 * queue tops are maintained atomics for the same reason. A racing release
 * yields a stale-but-plausible answer, which is the tolerance the value-free
 * doorbell already rests on.
 *
 * Cost: one relaxed load and a compare when the thread holds nothing, which is
 * 98.9 percent of picks in the only test that exercises mutexes at all and 100
 * percent everywhere else. Otherwise one iteration per resource actually held,
 * measured at 1.01 resources per holder.
 */
[[nodiscard]] std::uint8_t urgency(thread_control_block const& tcb) noexcept;

/**
 * @brief Ask @p tcb's core to reconsider what it is running.
 *
 * A pure hint carrying nothing. Used when a thread's urgency has RISEN because a
 * waiter arrived on something it holds: the new urgency is already visible in
 * that resource's queue top, but the owning core will not notice until it picks
 * again. Losing this costs latency, never correctness, because the next pick for
 * any reason folds the same truth.
 *
 * Only boosts need it. A de-boost that is observed late merely means the holder
 * ran at its old urgency slightly longer, which is safe.
 */
void request_repick(thread_control_block& tcb);


} // namespace cyros::thread_action

#endif // CYROS_THREAD_ACTION_HPP
