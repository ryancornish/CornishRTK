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

#include "base_mutex_access.hpp"
#include "scheduler.hpp"
#include "waitable_utilities.hpp"
#include "threading_subsystem.hpp"

#include <algorithm>
#include <bit>

namespace cyros
{

namespace
{

/* How far a transitive donation is followed before the answer is taken to be
 * maximally urgent.
 *
 * Real chains are short: the deepest the suite builds is 6, and the flagship PI
 * test is 2. The budget is not a performance knob, it is what stops a wait-for
 * cycle (a deadlocked application) from being walked forever. Exhausting it
 * yields 0, which over-boosts, which is the safe direction. */
inline constexpr unsigned max_inheritance_depth = 8;

/**
 * @brief min(base, best waiter of every base_mutex held).
 *
 * The definition of effective priority, folded from current truth.
 *
 * Evaluable by any core at any time and with no pi_lock: held_slots is a bounded
 * array of atomics and the queue tops are maintained atomics for exactly that.
 * The one restriction is on queue locks, see urgency_at.
 */
[[nodiscard]] std::uint8_t donated_floor(thread_control_block const& target, unsigned const depth) noexcept
{
   // Fast path: holds nothing, so urgency is base priority by definition. This
   // is the overwhelming majority of calls, measured at 98.9 percent of picks in
   // the only test in the suite that exercises mutexes at all, and 100 percent
   // everywhere else.
   auto mask = target.held_mask.load(std::memory_order_acquire);
   if (mask == 0) return target.base_priority;

   std::uint8_t floor = target.base_priority;
   while (mask != 0) {
      auto const slot = static_cast<std::size_t>(std::countr_zero(mask));
      mask &= static_cast<std::uint8_t>(mask - 1); // clear lowest set bit

      auto* held = target.held_slots[slot].load(std::memory_order_acquire);
      if (held == nullptr) continue; // bit was stale, see the ordering note on held_mask
      floor = std::min(base_mutex_access::urgency_contribution(*held, depth), floor);
   }
   return floor;
}

/**
 * @brief Applies the caller's reschedule_policy to a completed wake.
 *
 * Governs only the LOCAL pend: cross-core wakes have already emitted their IPI via the
 * ready path before this runs, so 'never' does not (and must not) suppress those.
 */
void apply_reschedule_policy(reschedule_policy const policy, schedule_hint const hint)
{
   if (policy == reschedule_policy::never) {
      return;
   }

   if (policy == reschedule_policy::always || hint == schedule_hint::warranted) {
      cyros_port_pend_reschedule();
   }
}

} // namespace

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

std::uint8_t wait_queue::top(std::atomic<thread_control_block*> const& resource_owner,
                             unsigned const depth,
                             inheritance_cache const& pi) const noexcept
{
   auto const cached = pi.top_priority.load(std::memory_order_acquire);

   // Fast path: no waiter here holds anything, so none can be more urgent than
   // its base priority and the cache is exact. This is almost every queue.
   if (!pi.may_have_bridges.load(std::memory_order_acquire)) return cached; // "acquire" pairs with store's "release"

   // Budget exhausted means a wait-for cycle, i.e. the application has
   // deadlocked. Answer conservatively instead of following it forever.
   if (depth == 0) return 0;

   // Snapshot under the lock, then release BEFORE recursing. Holding it across
   // the recursion would nest queue locks around the wait-for graph.
   static constexpr std::size_t max_snapshot = 4;
   std::array<thread_control_block*, max_snapshot> bridges{};
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
         // Re-derive from the current truth (stale answers are plausible and err toward over-boost.)
         if (n->owner->holds_nothing()) continue;
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
   if (overflow) return 0; // Over-boost, the safe direction

   std::uint8_t best = cached;
   for (std::size_t i = 0; i < found; ++i) {
      best = std::min(best, urgency_at(*bridges[i], depth - 1));
   }
   return best;
}

void wait_queue::arm(wait_node& node, inheritance_cache* pi) noexcept
{
   spinlock_guard guard(lock);

   CYROS_ASSERT(node.owner != nullptr);
   CYROS_ASSERT(node.next  == nullptr); // node must not already be on a list

   link(node);

   if (pi != nullptr && !node.owner->holds_nothing()) {
      // Adding a bridge, set the sticky hint
      pi->may_have_bridges.store(true, std::memory_order_release); // "release" pairs with load's "acquire"
   }
   refresh_top(pi);
}

/**
 * @brief Remove an armed node, idempotent against a racing wake.
 * @return true when the queue's best-waiter priority changed. Nothing acts on
 *         this today: leaving a queue can only LOWER a holder's urgency, and a
 *         de-boost needs no prompt because urgency is folded at the point of
 *         use. Kept because it is free and a caller that needs to know a top
 *         moved has no other way to find out.
 */
void wait_queue::disarm(wait_node& node, inheritance_cache* pi) noexcept
{
   spinlock_guard guard(lock);

   if (!unlink(node)) return;
   refresh_top(pi);
}

void wait_queue::refresh_top(inheritance_cache* pi) noexcept
{
   if (pi == nullptr) return; // Nothing reads it, so do not maintain it

   // Requires the queue lock held. Release pairs with the acquire in top() so
   // a fold prompted by a queue change (via a doorbell, or by reaching this
   // queue through a bridge) observes the value that change produced.
   pi->top_priority.store(head != nullptr ? head->owner->base_priority : inheritance_cache::no_waiter,
                          std::memory_order_release);

   // No more waiters, so we can safely clear the sticky hint
   if (head == nullptr) {
      pi->may_have_bridges.store(false, std::memory_order_release); // "release" pairs with load's "acquire"
   }
}

void wait_queue::wake_one(reschedule_policy policy, inheritance_cache* pi) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      if (head == nullptr) return;
      wait_node* node = head;
      chosen = node->owner;
      head = node->next;
      node->next = nullptr;
      refresh_top(pi);
   }

   schedule_hint hint = global_ready_thread(*chosen);
   apply_reschedule_policy(policy, hint);
}

void wait_queue::wake_all(reschedule_policy policy, inheritance_cache* pi) noexcept
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
         refresh_top(pi);
      }
      schedule_hint hint = global_ready_thread(*chosen);
      if (hint == schedule_hint::warranted) {
         aggregate_hint = schedule_hint::warranted;
      }
   }

   apply_reschedule_policy(policy, aggregate_hint);

   cyros_port_preempt_enable(token);
}

bool wait_queue::wake_one_and_commit(commit_fn const& commit, reschedule_policy policy, inheritance_cache* pi) noexcept
{
   thread_control_block* chosen = nullptr;
   {
      spinlock_guard guard(lock);

      if (head != nullptr) {
         wait_node* node = head;
         chosen = node->owner;
         head = node->next;
         node->next = nullptr;
         refresh_top(pi);
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
   schedule_hint hint = global_ready_thread(*chosen);
   apply_reschedule_policy(policy, hint);
   return true;
}

bool wait_queue::wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy, inheritance_cache* pi) noexcept
{
   return wake_one_and_commit(
      [&transfer](thread_control_block* chosen) {
         transfer(chosen != nullptr ? chosen->id : 0);
      },
      policy, pi);
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



std::uint8_t urgency(thread_control_block const& tcb) noexcept
{
   return donated_floor(tcb, max_inheritance_depth);
}

std::uint8_t urgency_at(thread_control_block const& tcb, unsigned const depth) noexcept
{
   return donated_floor(tcb, depth);
}

void commit_to_block(thread_control_block& tcb)
{
   auto token = cyros_port_preempt_disable();

   // Atomic CAS prevents a race between a blocking thread and an ISR.
   // Hazard: Without RMW, the thread could read 'prepared', an ISR could
   // clear the disposition, and the thread would blindly overwrite it to 'committed'.
   auto expected = thread_disposition::prepared;
   if (tcb.disposition.compare_exchange(expected, thread_disposition::committed)) {
      cyros_port_pend_reschedule(); // Delayed until preempt_enable()
   }

   cyros_port_preempt_enable(token);
}

void request_repick(thread_control_block& tcb)
{
   /* Prompt every core that could now want to run something different, by
    * walking the wait-for chain rather than shouting at all of them.
    *
    * The donor knows only the holder it failed to CAS against. If that holder is
    * itself blocked, its urgency rose too, and so did the urgency of whatever
    * holds the resource IT is parked on, all the way down. The thread that can
    * actually act is at the far end, and its core has no other reason to pick.
    *
    * blocked_on is what makes that chain walkable. Every read in here is a fact
    * whose staleness costs a misdirected or a missing prompt and never a wrong
    * value, so no lock is taken and nothing is retried: a lost prompt is
    * repaired by the next reschedule on that core, exactly as a doorbell is
    * allowed to be.
    *
    * Every owner met is prompted, not just the last one. That costs at most one
    * hint per hop and it is what keeps a stale state read from silently dropping
    * the prompt that mattered. Bounded by the same depth budget the fold uses,
    * so a wait-for cycle in the application terminates the walk instead of
    * spinning in it. */
   auto const this_core = cyros_port_get_core_id();
   auto* target = &tcb;

   for (unsigned hop = 0; hop < max_inheritance_depth; ++hop) {
      if (target->pinned_core == this_core) {
         cyros_port_pend_reschedule();
      } else {
         cyros_port_send_reschedule_ipi(target->pinned_core);
      }

      // Deliberately no thread_state check before continuing. blocked_on is
      // published while the waiter is armed, whether or not the scheduler has
      // parked it yet, so a ready or still-running waiter genuinely has
      // somewhere to pass this on. Gating on state == blocked here once
      // reintroduced an unbounded inversion: a waiter preempted between arming
      // and parking is rotated out ready with blocked_on set, it does not
      // re-donate when it resumes, and stopping the walk at it silences the
      // far end of the chain, which is the thread the walk exists to reach.
      auto* const next_resource = target->blocked_on.load(std::memory_order_acquire);
      if (next_resource == nullptr) return;

      auto* const next = base_mutex_access::holder_of(*next_resource);
      if (next == nullptr || next == target) return;

      target = next;
   }
}

/* The caller side of the two-phase block this file opens by describing. Lives
 * here for its access: it is friended by both waitable and wait_queue, and it
 * drives wait_node_vector and waitable_arm_guard. */
namespace this_thread
{

[[nodiscard]] std::size_t wait_on_any(std::span<waitable_ref> waitables) noexcept
{
   CYROS_ASSERT(!waitables.empty());
   CYROS_ASSERT_OP(waitables.size(), <=, config::max_wait_nodes);

   auto& tcb = scheduler_for_this_core().get_current_thread();
   wait_node_vector nodes(waitables.size(), tcb);

   while (true) {
      tcb.disposition = thread_disposition::prepared;
      std::optional<std::size_t> chosen;

      {
         // All waitable wakes are serialised on the arm_guard.
         // If a wake fires BEFORE the arm_guard:
         // - Thread is not readied because we are not registered.
         waitable_arm_guard arm_guard(waitables, nodes);
         // If a wake fires AFTER the arm_guard:
         // - Thread is readied and we are no longer 'prepared' to block.

         // Lowest-index wins on ties
         for (std::size_t i = 0; waitable& waitable : waitables) {
            if (waitable.try_satisfy()) {
               tcb.disposition = thread_disposition::none;
               chosen = i;

               // Leave the queue we just acquired from IMMEDIATELY, rather than
               // at the guard below. An owner still armed in its own resource's
               // queue is a cycle in the wait-for graph: the fold reads that
               // queue, finds the owner listed as a bridge, and recurses back
               // into the owner until the depth budget runs out and answers 0.
               // That is not the harmless over-boost it looks like. A thread
               // folding to 0 can never be beaten by set_thread_ready's
               // urgency < current comparison, so its core stops raising
               // preemption hints entirely while the window is open.
               //
               // Safe to do early: we own it, so no release can transfer it to
               // us in the meantime, which is the only thing the other nodes
               // have to stay armed for.
               waitable.queue.disarm(nodes[i]);
               break;
            }
            ++i;
         }

         if (!chosen) {
            // Nothing was satisfied. Park, unless a wake beat us to it.
            commit_to_block(tcb);
         }
      } // arm_guard: disarm all (and de-boost any holder whose top we lowered)

      if (chosen) {
         // The sweep must run here, AFTER the disarm above: while any node of
         // ours was still armed, a racing release could commit a transfer to
         // us. With every node off every queue no further transfer can choose
         // us, so the set of in-flight assignments is frozen and the ones we
         // are not returning are handed back. The contract this enforces: on
         // return the caller owns exactly waitables[chosen], plus whatever it
         // already owned before the call.
         for (std::size_t i = 0; waitable& waitable : waitables) {
            if (i != chosen) {
               waitable.renounce_if_assigned(tcb.id);
            }
            ++i;
         }
         return *chosen;
      }
   }
}

} // namespace this_thread

} // namespace cyros
