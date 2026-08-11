/**
 * @file base_mutex.cpp
 * @brief The kernel mutex: ownership, inheritance, and its own blocking loop.
 *
 * The blocking loop here is deliberately NOT shared with wait_on_any. Only the
 * commit step is, via thread_action::commit_to_block, because that is the part
 * where a lost wakeup hides. Arming one source and arming a span differ enough
 * that a common abstraction would cost more readability than it saves.
 */

#include <cyros/kernel/base_mutex.hpp>
#include <cyros/port/port.h>

#include "thread_action.hpp"
#include "threading_subsystem.hpp"

namespace cyros
{

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
 */
void base_mutex::claim_slot(thread_control_block& tcb) noexcept
{
   std::size_t slot = max_held_per_thread;
   for (std::size_t i = 0; i < max_held_per_thread; ++i) {
      base_mutex* expected = nullptr;
      // CAS orders any competing calls from the two contexts
      if (tcb.held_slots[i].compare_exchange_strong(expected, this,
                                                    std::memory_order_acq_rel,
                                                    std::memory_order_relaxed)) {
         slot = i;
         break;
      }
   }
   CYROS_ASSERT_OP(slot, <, max_held_per_thread); // Slot claim attempt failed!

   held_slot = static_cast<std::uint8_t>(slot);
   // The slot pointer is published by the CAS above, so setting the bit second
   // means a reader that sees the bit can always see the pointer.
   tcb.held_mask.fetch_or(static_cast<std::uint8_t>(1U << slot),
                          std::memory_order_release);
}

void base_mutex::retire_held(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(held_slot, <, max_held_per_thread); // Not registered to its owner
   CYROS_ASSERT(tcb.held_slots[held_slot].load(std::memory_order_relaxed) == this);

   // Clear the bit BEFORE the slot, the mirror of registration. A reader that
   // still sees the bit finds a slot that is either this resource (about to be
   // released, so a stale-but-plausible answer) or nullptr, and skips.
   tcb.held_mask.fetch_and(static_cast<std::uint8_t>(~(1U << held_slot)),
                           std::memory_order_release);
   tcb.held_slots[held_slot].store(nullptr, std::memory_order_release);
   held_slot = not_held;
}

void base_mutex::register_held(thread_control_block& tcb) noexcept
{
   spinlock_guard guard(tcb.pi_lock);

   // Idempotent: a re-poll after a spurious wake can land here for a resource
   // the earlier round already registered, and a handover registers on the
   // caller's behalf before it ever runs. Neither may occupy a second slot.
   if (held_slot != not_held) {
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
      thread_action::request_repick(*expected);
   }
   return false;
}

bool base_mutex::try_lock() noexcept
{
   auto& tcb = thread_action::get_current_thread_on_this_core();

   thread_control_block* expected = nullptr;
   if (!owner.compare_exchange_strong(expected, &tcb, std::memory_order_acq_rel)) {
      return false;
   }
   register_held(tcb);
   return true;
}

void base_mutex::lock() noexcept
{
   auto& tcb = thread_action::get_current_thread_on_this_core();

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
      thread_action::commit_to_block(tcb);
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
   auto& tcb = thread_action::get_current_thread_on_this_core();
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
