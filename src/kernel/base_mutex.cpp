/**
 * @file base_mutex.cpp
 * @brief The kernel mutex: ownership, inheritance, and its own blocking loop.
 *
 * The blocking loop here is deliberately NOT shared with wait_on_any. Only the
 * commit step is, via commit_to_block, because that is the part
 * where a lost wakeup hides. Arming one source and arming a span differ enough
 * that a common abstraction would cost more readability than it saves.
 */

#include <cyros/kernel/base_mutex.hpp>
#include <cyros/port/port.h>

#include "scheduler.hpp"
#include "threading_subsystem.hpp"

#include <algorithm>

namespace cyros
{

std::uint8_t base_mutex::urgency_contribution(unsigned const depth) const noexcept
{
   auto const from_queue = queue.top(owner, depth);
   if (uses_ceiling()) return std::min(ceiling_priority, from_queue);
   return from_queue;
}

thread_control_block* base_mutex::holder() const noexcept
{
   return owner.load(std::memory_order_acquire);
}

/**
 * @brief Assert invariants of a destroyed mutex
 */
base_mutex::~base_mutex()
{
   CYROS_ASSERT(owner.load(std::memory_order_relaxed) == nullptr); // Still owned
   CYROS_ASSERT(queue.empty()); // Destroyed with waiters parked on it
}

/**
 * @brief File this resource in tcb held_slots.
 * @param tcb Targeted thread
 *
 * Executed from two contexts:
 * - The owner registering its own acquisition under pi_lock
 * - a _releasing_ core committing a handover under a queue lock, where the new
 *   owner's pi_lock is out of reach
 *
 * The two contexts never run concurrently for one mutex. Only the context
 * that just made tcb the owner can be here, and the owner word transitions
 * through nullptr or a committed handover between owners, so claims for THIS
 * mutex are serialised by ownership itself. What the CAS below arbitrates is
 * different: concurrent claims for OTHER mutexes racing over the same
 * thread's free slots.
 */
void base_mutex::claim_slot(thread_control_block& tcb) noexcept
{
   std::size_t slot = max_held_per_thread;
   for (std::size_t i = 0; i < max_held_per_thread; ++i) {
      base_mutex* expected = nullptr;
      if (tcb.held_slots[i].compare_exchange_strong(expected, this,
                                                    std::memory_order_acq_rel,
                                                    std::memory_order_relaxed)) {
         slot = i;
         break;
      }
   }
   CYROS_ASSERT_OP(slot, <, max_held_per_thread); // Slot claim attempt failed!

   held_slot.store(static_cast<std::uint8_t>(slot), std::memory_order_relaxed);
   // The slot pointer is published by the CAS above, so setting the bit second
   // means a reader that sees the bit can always see the pointer.
   tcb.held_mask.fetch_or(static_cast<std::uint8_t>(1U << slot),
                          std::memory_order_release);
}

void base_mutex::retire_held(thread_control_block& tcb) noexcept
{
   auto const slot = held_slot.load(std::memory_order_relaxed);
   CYROS_ASSERT_OP(slot, <, max_held_per_thread); // Not registered to its owner
   CYROS_ASSERT(tcb.held_slots[slot].load(std::memory_order_relaxed) == this);

   // Clear the bit BEFORE the slot, the mirror of registration. A reader that
   // still sees the bit finds a slot that is either this resource (about to be
   // released, so a stale-but-plausible answer) or nullptr, and skips.
   tcb.held_mask.fetch_and(static_cast<std::uint8_t>(~(1U << slot)),
                           std::memory_order_release);
   tcb.held_slots[slot].store(nullptr, std::memory_order_release);
   held_slot.store(not_held, std::memory_order_relaxed);
}

void base_mutex::register_held(thread_control_block& tcb) noexcept
{
   spinlock_guard guard(tcb.pi_lock);

   // Idempotent: a re-poll after a spurious wake can land here for a resource
   // the earlier round already registered, and a handover registers on the
   // caller's behalf before it ever runs. Neither may occupy a second slot.
   if (held_slot.load(std::memory_order_relaxed) != not_held) {
      return;
   }

   claim_slot(tcb);
}

/**
 * @brief Take if free, recognise ownership already transferred to us, or donate.
 *
 * @param tcb
 * @return
 */
bool base_mutex::acquire_condition(thread_control_block& tcb) noexcept
{
   thread_control_block* expected = nullptr;
   if (owner.compare_exchange_strong(expected, &tcb, std::memory_order_acq_rel)) {
      register_held(tcb);
      return true; // Free for taking
   }

   if (expected == &tcb) {
      // Ownership was transferred to us while parked, and the releaser already
      // filed the resource in our held slots on our behalf. Nothing left to do.
      return true;
   }

   // About to park behind a live owner: donate our urgency to the owner.
   //
   // expected is the owner the failed CAS above observed, so no second load is
   // needed. It races the owner terminating, but an owner must not terminate
   // while holding a resource (asserted at teardown), so a live read here is
   // part of that same contract.
   if (expected != nullptr) {
      request_repick(*expected);
   }
   return false;
}

/**
 * @brief Enforce the ceiling contract, on every path that can acquire.
 *
 * A ceiling less urgent than the acquirer's own base priority is not a tuning
 * mistake, it is the protocol inverted: the holder would run BELOW a thread
 * that can be blocked by it, which is the exact inversion the ceiling exists to
 * prevent. Caught here rather than absorbed by folding the queue as well,
 * because absorbing it would hide the misconfiguration and cost every ceiling
 * acquire a queue read. POSIX rejects the same case with EINVAL.
 */
void base_mutex::check_ceiling_contract(thread_control_block const& tcb) const noexcept
{
   if (!uses_ceiling()) return;
   CYROS_ASSERT_OP(ceiling_priority, <=, tcb.base_priority); // ceiling below locker
}

bool base_mutex::try_lock() noexcept
{
   auto& tcb = scheduler_for_this_core().get_current_thread();
   check_ceiling_contract(tcb);

   thread_control_block* expected = nullptr;
   if (!owner.compare_exchange_strong(expected, &tcb, std::memory_order_acq_rel)) {
      return false;
   }
   register_held(tcb);
   return true;
}

void base_mutex::lock() noexcept
{
   auto& tcb = scheduler_for_this_core().get_current_thread();
   check_ceiling_contract(tcb);

   wait_queue::wait_node node{};
   node.owner = &tcb;

   while (true) {
      tcb.disposition = thread_disposition::prepared;

      // Arm BEFORE polling. A release that lands after this point is guaranteed
      // to see us, and our urgency is already in the queue's top, so the donate
      // below prompts a holder that can already observe the boost.
      //
      // An owner must never appear in its own queue: the fold would reach this
      // queue, find us listed as a waiter, and recurse back into our own urgency
      // until its depth budget ran out and answered 0. Both exits from the loop
      // below disarm before returning, and the handover check after the park is
      // what keeps a woken owner from coming back round to re-arm.
      CYROS_ASSERT(owner.load(std::memory_order_relaxed) != &tcb); // Owner queued on itself
      queue.arm(node);

      // Publish the wait-for edge AFTER arming and BEFORE polling, so it is set
      // exactly while we are queued. A prompt walk that reaches us can then
      // follow it to whoever holds this resource, which is what lets a
      // transitive donation reach the far end of a chain without broadcasting.
      tcb.blocked_on.store(this, std::memory_order_release);

      if (acquire_condition(tcb)) {
         tcb.blocked_on.store(nullptr, std::memory_order_release);
         tcb.disposition = thread_disposition::none;
         // Leave our own queue immediately. An owner still armed in its own
         // resource's queue is a cycle in the wait-for graph, and the fold would
         // recurse back into us until its budget ran out and answered 0. See
         // pi-derived-urgency-proposal.md 15b and 15c for why that is not the
         // harmless over-boost it looks like.
         queue.disarm(node);
         return;
      }

      // Park, unless a wake already revoked the intent. Control resumes here
      // once something readies us, which for a mutex means a handover.
      commit_to_block(tcb);
      queue.disarm(node);
      tcb.blocked_on.store(nullptr, std::memory_order_release);

      // Recognise a handover BEFORE looping, so an owner never re-arms on its
      // own queue.
      if (owner.load(std::memory_order_acquire) == &tcb) {
         tcb.disposition = thread_disposition::none;
         return;
      }
   }
}

void base_mutex::unlock(reschedule_policy policy) noexcept
{
   auto& tcb = scheduler_for_this_core().get_current_thread();
   CYROS_ASSERT_OP(owner.load(std::memory_order_relaxed), ==, &tcb); // Release by non-owner

   // Retire from the held slots FIRST, so a fold running during the handover no
   // longer counts this resource's waiters against us. Correct because we are
   // giving it up, and its waiters' urgency is about to become the next owner's
   // concern.
   {
      spinlock_guard guard(tcb.pi_lock);
      retire_held(tcb);
   }

   hand_over(policy);

   /* A ceiling release needs a prompt that an inheritance release does not.
    *
    * Inheritance only ever boosts under contention, so a release with no waiter
    * never boosted us and there is nothing to give back. A ceiling boosts on an
    * UNCONTENDED acquire, so releasing it can drop our urgency with no waiter to
    * wake and therefore nothing to pend a reschedule. A thread that was held off
    * by the ceiling would then sit ready behind us until some unrelated
    * reschedule arrived, which on a tickless build may be never.
    *
    * Unconditional rather than conditional on having actually been preempted:
    * the pend coalesces, and the alternative is reconstructing what the ceiling
    * held off, which is exactly the cached-derived-value shape this design
    * refuses. */
   if (uses_ceiling() && policy != reschedule_policy::never) {
      cyros_port_pend_reschedule();
   }
}

/**
 * @brief The release commit: hand to the best waiter or free, under the queue lock.
 *
 * @param policy
 */
void base_mutex::hand_over(reschedule_policy policy) noexcept
{
   queue.wake_one_and_commit(
      [this](thread_control_block* chosen) {
         owner.store(chosen, std::memory_order_release);
         if (!chosen) return;

         // File the resource in the new owner's slots HERE, not in its own
         // next poll. Ownership that is not in held_slots is invisible to
         // both the pick's routing and the urgency fold, so deferring it
         // leaves the new owner unboostable for as long as it takes to run,
         // which a mid-priority thread on its core can make unbounded.
         //
         // No pi_lock is taken, so the queue-lock -> pi_lock nesting the lock
         // order forbids does not arise. claim_slot uses a CAS for exactly
         // that reason.
         claim_slot(*chosen);
      },
      policy);
}

} // namespace cyros
