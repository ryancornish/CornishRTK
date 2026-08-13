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
 * @brief Make a thread runnable on its pinned core (cross-core-safe).
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
schedule_hint global_ready_thread(thread_control_block& tcb);

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
 * @brief urgency() with an explicit recursion budget.
 *
 * A boosted holder can itself be blocked on something it does not own, whose
 * queue contains another holder, and so on: that chain is transitive priority
 * inheritance. Following it needs a bound, because a wait-for CYCLE is a
 * deadlocked application and the kernel must not walk one forever.
 *
 * CALLER MUST NOT HOLD A QUEUE LOCK. The recursion reaches wait_queue::top,
 * which takes one to snapshot the bridges parked on that queue. It releases it
 * before recursing, so queue locks are never NESTED and a wait-for cycle is
 * merely observed rather than deadlocked on, but that only holds while every
 * caller enters from outside. In particular nothing inside a wake_one_and_commit
 * callback may ask for a thread's urgency, and thread::get_priority() is exactly
 * that question.
 *
 * held_slots is a bounded array of atomics so the fold itself needs no lock, and
 * a torn read yields a stale-but-plausible urgency, the same tolerance the rest
 * of this design rests on.
 */
[[nodiscard]] std::uint8_t urgency_at(thread_control_block const& tcb, unsigned depth) noexcept;

/**
 * @brief Commit to parking, unless a wake has already revoked the intent.
 *
 * The subtle half of the two-phase block, and the ONLY part of it that is worth
 * sharing between the group wait and a primitive with its own blocking loop. The
 * arming loops differ enough (one source versus many) that a shared abstraction
 * would cost more readability than it saves, but this test-and-commit must be
 * identical everywhere or the lost-wakeup window reopens.
 *
 * Preemption is disabled across the test and the store, so a wake cannot land
 * between deciding to block and recording it. A wake that arrived earlier has
 * already cleared the disposition, and that is what this re-reads: seeing
 * anything other than prepared means someone readied us after we armed, so we
 * must not park. The pend is deferred to preempt_enable by construction.
 *
 * Caller must be armed on every source it intends to wake from BEFORE calling.
 */
void commit_to_block(thread_control_block& tcb);

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
