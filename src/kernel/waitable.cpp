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
 * Block-on-any uses a SEPARATE wait_node per source, allocated on the
 * blocking thread's stack. A wait_node has exactly one 'next' pointer; it
 * cannot be in two intrusive lists at once. The TCB's embedded wait_node
 * serves single-wait blocks at zero extra cost; block_on_any pays N stack
 * nodes for N sources, with no heap and no pool.
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

std::uint8_t wait_queue::top(unsigned const depth) const noexcept
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
      for (auto* n = head; n != nullptr; n = n->next) {
         if (!n->counted_as_bridge) continue;
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
 * @return true when the queue's best-waiter priority changed, meaning a
 *         holder's inheritance is now stale and must be re-derived. The
 *         caller chases that, disarm itself takes no pi_lock.
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
   // a recompute that learns of a queue change (via a doorbell or its own
   // reslot return) observes the value that change produced.
   top_priority.store(head != nullptr ? head->owner->base_priority : no_waiter,
                      std::memory_order_release);
}

bool wait_queue::reslot(wait_node& node) noexcept
{
   spinlock_guard guard(lock);

   if (!unlink(node)) return false;
   link(node);

   std::uint8_t const old_top = top_priority.load(std::memory_order_relaxed);
   refresh_top();
   return top_priority.load(std::memory_order_relaxed) != old_top;
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


/* ============================================================================
 * pi_waitable
 * ========================================================================= */

pi_waitable::~pi_waitable()
{
   CYROS_ASSERT(owner.load(std::memory_order_relaxed) == nullptr); // Resource still owned by a thread
}

void pi_waitable::register_held(thread_control_block& tcb) noexcept
{
   {
      spinlock_guard guard(tcb.pi_lock);

      // Idempotent: a wait_condition re-poll after a spurious wake can land
      // here for a resource the earlier round already registered, and must not
      // occupy a second slot.
      if (held_slot != not_held) {
         return;
      }

      std::size_t free_slot = max_held_per_thread;
      for (std::size_t i = 0; i < max_held_per_thread; ++i) {
         if (tcb.held_slots[i].load(std::memory_order_relaxed) == nullptr) {
            free_slot = i;
            break;
         }
      }
      // Hard error, never a silent drop: an unrecorded held resource is a lost
      // donation, which is the failure class this representation exists to remove.
      CYROS_ASSERT_OP(free_slot, <, max_held_per_thread); // max_held_per_thread exceeded

      held_slot = static_cast<std::uint8_t>(free_slot);
      // Publish the slot BEFORE the bit. A reader that sees the bit must be able
      // to see the pointer, so the bit is what makes the slot visible.
      tcb.held_slots[free_slot].store(this, std::memory_order_release);
      tcb.held_mask.fetch_or(static_cast<std::uint8_t>(1U << free_slot),
                             std::memory_order_release);
   }

   // Nothing to propagate. Waiters already queued at acquisition time (the
   // uncontended CAS can win while others are parked, and a transferred owner
   // can inherit waiters) are picked up automatically, because urgency is folded
   // from the queue tops at the point of use rather than pushed to a cache here.
}

bool pi_waitable::pi_try_acquire() noexcept
{
   auto& tcb = thread_action::get_current_thread_on_this_core();

   thread_control_block* expected = nullptr;
   if (!owner.compare_exchange_strong(expected, &tcb, std::memory_order_acq_rel)) {
      return false;
   }
   register_held(tcb);
   return true;
}

bool pi_waitable::pi_acquire_condition(thread& caller) noexcept
{
   // The wait_condition contract guarantees caller is the running thread on
   // this core, and the kernel's view of it carries the TCB the public
   // handle cannot expose.
   (void)caller;
   auto& tcb = thread_action::get_current_thread_on_this_core();

   thread_control_block* expected = nullptr;
   if (owner.compare_exchange_strong(expected, &tcb, std::memory_order_acq_rel)) {
      register_held(tcb);
      return true; // free, taken uncontended
   }

   if (expected == &tcb) {
      // Ownership was transferred to us while parked. The releaser committed
      // the ownership word under the queue lock but deliberately did NOT
      // touch our held list: linkage takes our pi_lock, and queue-lock ->
      // pi_lock nesting is forbidden because the recompute nests them the
      // other way round. The handover is completed here, in our own context.
      // Any inversion this defers is nil, the transfer chose the BEST waiter,
      // so no remaining waiter is more urgent than us in the gap.
      register_held(tcb);
      return true;
   }

   // About to park behind a live owner: donate. The doorbell carries no
   // priority value, only "recompute from current truth", so a ring that goes
   // stale in flight (the owner released, or another donor got there first)
   // degrades to a redundant recompute rather than a wrong answer. Our own
   // urgency is visible to that recompute because we armed before polling,
   // the queue's top already includes us.
   //
   // expected is the owner the failed CAS above observed, so no second load is
   // needed. It races the owner terminating, but an owner must not terminate
   // while holding a pi resource (asserted at teardown), so a live read here is
   // part of that same contract.
   // The donation itself needs no message: we armed before polling, so this
   // resource's top() already reflects our urgency and the holder's core folds
   // it in at its next pick. What the holder's core does need is a REASON to
   // pick again, since a boost that nobody re-evaluates has no effect until
   // something else reschedules. That is a pure hint: lose it and the boost
   // lands at the next reschedule for any cause.
   if (expected != nullptr) {
      thread_action::request_repick(*expected);
   }
   return false;
}

void pi_waitable::pi_release(reschedule_policy policy) noexcept
{
   auto& tcb = thread_action::get_current_thread_on_this_core();
   CYROS_ASSERT_OP(owner.load(std::memory_order_relaxed), ==, &tcb); // release by non-owner

   // Retire from the held list FIRST, so the restore recompute below no
   // longer counts this resource's waiters against us. A donor ringing in
   // this window recomputes us without this resource, which is correct, we
   // are giving it up and its waiters' urgency is about to become the next
   // owner's concern.
   {
      spinlock_guard guard(tcb.pi_lock);

      CYROS_ASSERT_OP(held_slot, <, max_held_per_thread); // resource not registered to its owner
      CYROS_ASSERT(tcb.held_slots[held_slot].load(std::memory_order_relaxed) == this); // slot mismatch

      // Clear the bit BEFORE the slot, the mirror of registration. A reader that
      // still sees the bit finds a slot that is either this resource (about to
      // be released, so a stale-but-plausible answer) or nullptr, and skips.
      tcb.held_mask.fetch_and(static_cast<std::uint8_t>(~(1U << held_slot)),
                              std::memory_order_release);
      tcb.held_slots[held_slot].store(nullptr, std::memory_order_release);
      held_slot = not_held;
   }

   // Hand over (or free) with the commit under the queue lock, closing the
   // lost-wakeup window exactly as wake_one_and_transfer documents.
   hand_over(policy);

   // No restore step. Our urgency stops including this resource the moment it
   // leaves held_slots, because nothing cached it. And a release that mattered
   // had waiters, so hand_over above already woke one and applied the
   // reschedule policy; a release with no waiters never boosted us at all.
}

void pi_waitable::hand_over(reschedule_policy policy) noexcept
{
   // The commit touches only this object's own atomics: held-list linkage for
   // the new owner is completed by the new owner itself in its next
   // wait_condition poll, keeping every pi_lock acquisition outside every
   // queue lock. One word means there is no write ordering to get right: a
   // donor either sees the new owner or the old one, never a mix.
   queue.wake_one_and_commit(
      [this](thread_control_block* chosen) {
         owner.store(chosen, std::memory_order_release);
      },
      policy);
}

void pi_waitable::renounce_if_assigned(thread::id const thread_id) noexcept
{
   auto* const assigned = owner.load(std::memory_order_acquire);
   if (assigned == nullptr || assigned->id != thread_id) {
      return; // never assigned to us, nothing to hand back
   }

   // Assigned versus already-owned is the load-bearing distinction here. The
   // ownership word alone cannot make it: a caller that held this resource
   // BEFORE entering the group wait also reads its own id. What separates
   // them is registration, an earlier acquisition linked us into the
   // caller's held list, an in-flight assignment did not (linkage happens in
   // the wait_condition poll the group wait never reached). Renouncing
   // registered ownership would put two threads in one critical section, so
   // it is kept.
   //
   // The read is safe unlocked: held_slot mutates only in the owner's own
   // context, and we ARE the owner's context.
   if (held_slot != not_held) {
      return; // registered ownership from before the wait, the caller keeps it
   }

   // No unlink (never linked) and no restore recompute (an unregistered
   // resource never contributed to our priority). Waiters parked behind the
   // assignment are honoured by the same barge-free commit as a release.
   hand_over(reschedule_policy::automatic);
}

thread_control_block* pi_waitable::donation_target(thread::id& expected_id) noexcept
{
   // Racy by design: the owner can change or vanish between this load and
   // the recompute acting on it. The doorbell's id check plus the value-free
   // recompute make a stale answer harmless.
   auto* const owner_snapshot = owner.load(std::memory_order_acquire);
   if (owner_snapshot != nullptr) {
      expected_id = owner_snapshot->id;
   }
   return owner_snapshot;
}

} // namespace cyros
