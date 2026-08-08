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
 * @brief Recompute a thread's effective priority from truth, chasing chains.
 *
 * Effective priority is derived state: min(base_priority, best queued waiter
 * of every pi_waitable the thread holds). This walk re-derives it for the
 * seed thread and then chases any transitive donation the change uncovered,
 * a boosted thread that is itself blocked on a pi_waitable re-orders that
 * queue, and if the queue's best waiter changed, ITS holder must be
 * re-derived too.
 *
 * Requests never carry priority values, only "recompute from truth". That is
 * what makes every form of staleness benign: two donors racing converge on
 * the same queue-head read, a restore racing a late boost recomputes to the
 * same answer, and a doorbell for a released resource finds it absent from
 * the held list. The one hazard idempotence cannot absorb is the TCB memory
 * being recycled by a new thread, which expected_id filters, ids are never
 * reused within a kernel session.
 *
 * Only the owning core mutates a thread's scheduling position, so remote
 * targets are forwarded as intake doorbells and drained in their scheduler.
 * Local targets are processed inline.
 * The walk is iterative rather than recursive because it can run on the
 * shared interceptor stack via drain_intake, and a user-constructed deadlock
 * cycle terminates naturally, a hop is only queued when a queue top actually
 * changed, and around a cycle the donated priority stops changing.
 *
 * Lock ordering: takes pi_lock then queue locks (reslot), never the reverse,
 * and never two pi_locks together, chain hops are queued and processed after
 * the current target's pi_lock is released.
 */
void recompute_thread_priority(thread_control_block& tcb, thread::id expected_id);

} // namespace cyros::thread_action

#endif // CYROS_THREAD_ACTION_HPP
