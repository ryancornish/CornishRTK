/**
 * @file waitable.cpp
 * @brief Implementation of the waitable kernel base class.
 *
 * Lost-wakeup correctness rests on two cooperating mechanisms:
 *
 *  1. The two-phase block in this file: arm (link a wait_node into the
 *     queue under the queue lock, set the thread's state to blocked), then
 *     park (yield to the scheduler). The queue lock is released between
 *     arm and park.
 *
 *  2. The scheduler's tolerance of state==ready on entry to reschedule().
 *     A wake that arrives in the window between arm-unlock and park sees a
 *     properly-linked, blocked thread; it unlinks the node and sets
 *     state=ready. When the thread then yields, reschedule() sees
 *     state==ready and treats it as a rotation (re-enqueue, pick the next
 *     thread). No yield is "lost"; no wake is dropped.
 *
 * A wait_node has exactly one 'next' pointer, so it cannot be in two intrusive
 * lists at once, which is why waiting on several sources needs one node per
 * source. The nodes live in a wait_node_vector on the blocking thread's own
 * stack, so there is no heap and no pool. Every wait goes through wait_on_any,
 * including a wait on a single source.
 */

#include <cyros/kernel/waitable.hpp>
#include <cyros/port/port.h>

#include "scheduler.hpp"
#include "thread_action.hpp"
#include "threading_subsystem.hpp"

namespace cyros
{


/**
 * @brief Applies the caller's reschedule_policy to a completed wake.
 *
 * Governs only the LOCAL pend: cross-core wakes have already emitted their IPI via the
 * ready path before this runs, so 'never' does not (and must not) suppress those.
 */
static void apply_reschedule_policy(reschedule_policy const policy, schedule_hint const hint)
{
   if (policy == reschedule_policy::never) {
      return;
   }

   if (policy == reschedule_policy::always || hint == schedule_hint::warranted) {
      cyros_port_pend_reschedule();
   }
}

/* ============================================================================
 * wait_queue
 * ========================================================================= */

bool wait_queue::empty() const noexcept
{
   // Advisory: not under lock. Fine for "should I bother waking" hints.
   return head == nullptr;
}

/**
 * @brief Priority-ordered insert (best at head), keyed on BASE priority.
 *
 * Base never changes, so a node's position is fixed for the life of the wait and
 * nothing ever has to re-slot it. That is what removed the whole reslot-and-chase
 * machinery: the old key was effective priority, which moved under the queue and
 * had to be chased.
 */
void wait_queue::link(wait_node& node) noexcept
{
   wait_node** slot = &head;
   while (*slot && (*slot)->owner->base_priority <= node.owner->base_priority) {
      slot = &(*slot)->next;
   }
   node.next = *slot;
   *slot  = &node;
}

/**
 * @brief Unlink node from wait_queue if present.
 * Idempotent if node not present.
 */
bool wait_queue::unlink(wait_node& node) noexcept
{
   wait_node** slot = &head;
   while (*slot && *slot != &node) {
      slot = &(*slot)->next;
   }
   if (*slot != &node) {
      return false;
   }
   *slot = node.next;
   node.next = nullptr;
   return true;
}

void wait_queue::drop_bridge(wait_node& node) noexcept
{
   if (!node.counted_as_bridge) return;
   node.counted_as_bridge = false;
   bridge_count.fetch_sub(1, std::memory_order_release);
}

std::uint8_t wait_queue::top(std::atomic<thread_control_block*> const& resource_owner,
                             unsigned const depth) const noexcept
{
   auto const cached = top_priority.load(std::memory_order_acquire);

   // Fast path: no waiter here holds anything, so none can be more urgent than
   // its base priority and the cache is exact. This is almost every queue.
   if (bridge_count.load(std::memory_order_acquire) == 0) return cached;

   // Budget exhausted means a wait-for cycle, i.e. the application has
   // deadlocked. Answer conservatively instead of following it forever.
   if (depth == 0) return 0;

   // Snapshot under the lock, then release BEFORE recursing. Holding it across
   // the recursion would nest queue locks around the wait-for graph.
   static constexpr std::size_t max_snapshot = 4;
   thread_control_block* bridges[max_snapshot];
   std::size_t found = 0;
   bool overflow = false;
   {
      spinlock_guard guard(const_cast<spinlock&>(lock));

      // Relaxed is enough here: the transition this read must not miss is the
      // handover, which stores the owner under this same lock, so the lock
      // orders it. See the header note for why the read must be under the
      // lock at all.
      auto* const holder = resource_owner.load(std::memory_order_relaxed);

      for (auto* n = head; n != nullptr; n = n->next) {
         if (!n->counted_as_bridge) continue;
         // The owner itself, still armed in its own queue between the winning
         // CAS and its disarm. Recursing into it would be a self-cycle that
         // exhausts the budget and answers 0. See the note on top()'s
         // resource_owner parameter for why that is not the harmless
         // over-boost it looks like.
         if (n->owner == holder) continue;
         if (found == max_snapshot) { overflow = true; break; }
         bridges[found++] = n->owner;
      }
   }
   if (overflow) return 0; // over-boost, the safe direction

   std::uint8_t best = cached;
   for (std::size_t i = 0; i < found; ++i) {
      best = std::min(best, thread_action::urgency_at(*bridges[i], depth - 1));
   }
   return best;
}

void wait_queue::arm(wait_node& node) noexcept
{
   spinlock_guard guard(lock);

   CYROS_ASSERT(node.owner != nullptr);
   CYROS_ASSERT(node.next  == nullptr); // node must not already be on a list

   link(node);
   node.counted_as_bridge = !node.owner->holds_nothing();
   if (node.counted_as_bridge) {
      bridge_count.fetch_add(1, std::memory_order_release);
   }
   refresh_top();
}

/**
 * @brief Remove an armed node, idempotent against a racing wake.
 * @return true when the queue's best-waiter priority changed. Nothing acts on
 *         this today: leaving a queue can only LOWER a holder's urgency, and a
 *         de-boost needs no prompt because urgency is folded at the point of
 *         use. Kept because it is free and a caller that needs to know a top
 *         moved has no other way to find out.
 */
bool wait_queue::disarm(wait_node& node) noexcept
{
   spinlock_guard guard(lock);

   if (!unlink(node)) return false;
   drop_bridge(node);

   std::uint8_t const old_top = top_priority.load(std::memory_order_relaxed);
   refresh_top();
   return top_priority.load(std::memory_order_relaxed) != old_top;
}

void wait_queue::refresh_top() noexcept
{
   // Requires the queue lock held. Release pairs with the acquire in top() so
   // a fold prompted by a queue change (via a doorbell, or by reaching this
   // queue through a bridge) observes the value that change produced.
   top_priority.store(head != nullptr ? head->owner->base_priority : no_waiter,
                      std::memory_order_release);
}

void wait_queue::wake_one(reschedule_policy policy) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      if (head == nullptr) return;
      wait_node* node = head;
      chosen = node->owner;
      head = node->next;
      node->next = nullptr;
      drop_bridge(*node);
      refresh_top();
   }

   schedule_hint hint = thread_action::ready_thread(*chosen);
   apply_reschedule_policy(policy, hint);
}

void wait_queue::wake_all(reschedule_policy policy) noexcept
{
   // Atomic batch admit. Preemption is held off for the entire batch so
   // every waiter lands on the ready matrix before any of them can run on this
   // core.
   schedule_hint aggregate_hint = schedule_hint::unwarranted;

   auto token = cyros_port_preempt_disable();

   while (true) {
      thread_control_block* chosen = nullptr;
      {
         spinlock_guard guard(lock);

         if (head == nullptr) break;
         wait_node* node = head;
         chosen = node->owner;
         head = node->next;
         node->next = nullptr;
         drop_bridge(*node);
         refresh_top();
      }
      schedule_hint hint = thread_action::ready_thread(*chosen);
      if (hint == schedule_hint::warranted) {
         aggregate_hint = schedule_hint::warranted;
      }
   }

   apply_reschedule_policy(policy, aggregate_hint);

   cyros_port_preempt_enable(token);
}

bool wait_queue::wake_one_and_commit(commit_fn const& commit, reschedule_policy policy) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      if (head != nullptr) {
         wait_node* node = head;
         chosen = node->owner;
         head = node->next;
         node->next = nullptr;
         drop_bridge(*node);
         refresh_top();
      }
      CYROS_ASSERT(chosen == nullptr || chosen->id != 0);

      // The commit for BOTH outcomes happens under the lock. Deciding the
      // empty case outside it would let a waiter arm, poll the still-held
      // resource, and park just before this release frees it, a lost wakeup
      // with no future wake to recover it.
      commit(chosen);
   }

   if (chosen == nullptr) {
      return false;
   }

   // The commit is already done, so readying outside the lock is pure
   // delivery. Only the TCB is touched out here. The wait_node lives on the
   // waiter's stack and is only dereferenced under the lock, matching the
   // wake_one discipline.
   schedule_hint hint = thread_action::ready_thread(*chosen);
   apply_reschedule_policy(policy, hint);
   return true;
}

bool wait_queue::wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy) noexcept
{
   return wake_one_and_commit(
      [&transfer](thread_control_block* chosen) {
         transfer(chosen != nullptr ? chosen->id : 0);
      },
      policy);
}

/* ============================================================================
 * waitable - public surface
 * ========================================================================= */

waitable::~waitable()
{
   // It is a programming error to destroy an waitable with parked waiters.
   CYROS_ASSERT(queue.empty());
}

void waitable::wake_one(reschedule_policy policy) noexcept
{
   queue.wake_one(policy);
}

void waitable::wake_all(reschedule_policy policy) noexcept
{
   queue.wake_all(policy);
}

bool waitable::wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy) noexcept
{
   return queue.wake_one_and_transfer(transfer, policy);
}


} // namespace cyros
