#include "scheduler.hpp"

#include "thread_action.hpp"

namespace cyros
{

void scheduler::pin_thread(thread_control_block& tcb)
{
   CYROS_ASSERT(core_id < CYROS_PORT_CORE_COUNT);
   tcb.pinned_core = core_id;
   pinned_thread_counter.fetch_add(1, std::memory_order_relaxed);
}

void scheduler::init_idle_thread()
{
   stack_layout slayout(idle_stack, 0);
   idle_thread = ::new (slayout.tcb) thread_control_block(
      config::max_priorities-1,
      core_affinity::from_id(core_id),
      slayout.user_stack,
      idle_task,
      nullptr
   );
   idle_thread->id = idle_thread_id;
   idle_thread->state = thread_state::ready;
   idle_thread->pinned_core = core_id;
}

// Core-local operations (only called on owning core)
void scheduler::start() noexcept
{
   CYROS_ASSERT(idle_thread != nullptr); // init_idle_thread() must run before start()

   auto* first = ready_matrix.pop_best_thread();
   if (first == nullptr) {
      first = idle_thread;
   }
   CYROS_ASSERT(first != nullptr);
   CYROS_ASSERT_OP(first->state, ==, thread_state::ready);

   set_thread_running(*first);

   //cyros_port_set_thread_pointer(current_thread);
   cyros_port_start_first(current_thread->context());
}

schedule_hint scheduler::set_thread_ready(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);

   // Interrupt-masked because this is the choke point every wake funnels
   // through, including ISR wakes: the matrix splice below must be atomic
   // against both a cross-core reschedule IPI landing mid-splice (a latent
   // race even in thread context) and an ISR wake re-entering it.
   this_core::critical_guard guard;

   // Idempotent: a remote core might have sent us a late request to ready
   // this thread, and we have already terminated it since. No-op.
   if (tcb.state == thread_state::terminated) {
      return schedule_hint::unwarranted;
   }

   // Idempotent: if already enqueued, this is a redundant wake/admit (e.g.
   // two signallers raced, or a stale wake from a prior round). The thread
   // is already going to run.
   if (tcb.is_enqueued()) {
      CYROS_ASSERT(tcb.state == thread_state::ready);
      return schedule_hint::unwarranted;
   }

   tcb.state = thread_state::ready;

   // Idle thread does not belong in the ready_matrix,
   // but DOES follow state transition semantics
   if (&tcb == idle_thread) {
      return schedule_hint::unwarranted;
   }

   ready_matrix.enqueue_thread(tcb);

   if (tcb.is_higher_priority_than(current_thread_priority())) {
      return schedule_hint::warranted;
   }
   return schedule_hint::unwarranted;
}

void scheduler::set_thread_running(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::ready);

   tcb.state = thread_state::running;
   current_thread = &tcb;
}

void scheduler::set_thread_blocked(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
   CYROS_ASSERT_OP(tcb.disposition, ==, thread_disposition::committed);
   CYROS_ASSERT(!tcb.is_enqueued());

   tcb.disposition = thread_disposition::none;
   tcb.state = thread_state::blocked;
}

void scheduler::set_thread_terminated(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
   CYROS_ASSERT(!tcb.is_enqueued());
   CYROS_ASSERT(tcb.holds_nothing()); // Thread cannot own a pi_waitable on termination

   tcb.state = thread_state::terminated;
   tcb.termination.terminate(); // signal joiners
}

schedule_hint scheduler::reprioritise_thread(thread_control_block& tcb, uint8_t const new_effective) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(new_effective, <, config::max_priorities);

   switch (tcb.state) {
      case thread_state::running:
         // A running thread is this core's current thread and lives outside
         // the matrix, so only the field moves. It re-enqueues at the new
         // value on its next rotation. A drop below a ready peer means the
         // scheduler now prefers that peer, which is exactly the restore
         // case ending an inversion, so flag it.
         CYROS_ASSERT(&tcb == current_thread);
         tcb.set_priority(new_effective);
         {
            auto const best = ready_matrix.best_priority();
            if (best >= 0 && static_cast<uint8_t>(best) < new_effective) {
               return schedule_hint::warranted;
            }
         }
         return schedule_hint::unwarranted;

      case thread_state::ready:
         if (tcb.is_enqueued()) {
            // Removal is keyed on the current field value, so the order here
            // is load-bearing: remove at old, write, re-enqueue at new.
            ready_matrix.remove_thread(tcb);
            tcb.set_priority(new_effective);
            ready_matrix.enqueue_thread(tcb);
            if (tcb.is_higher_priority_than(current_thread_priority())) {
               return schedule_hint::warranted;
            }
         } else {
            // Readied but not yet admitted (in-flight inbox request, or the
            // idle thread). The eventual enqueue reads the new value.
            tcb.set_priority(new_effective);
         }
         return schedule_hint::unwarranted;

      case thread_state::blocked:
      case thread_state::created:
         // Position in any wait queues is the caller's (the recompute walk's)
         // responsibility, only the field moves here.
         tcb.set_priority(new_effective);
         return schedule_hint::unwarranted;

      case thread_state::terminated:
         return schedule_hint::unwarranted;
   }

   return schedule_hint::unwarranted;
}

void scheduler::drain_inbox() noexcept
{
   inbox_poke_pending.store(false, std::memory_order_release);

   cross_core_request request;
   while (inbox.pop(request)) {
      // Release BEFORE servicing, so a request raised while we are servicing
      // this one queues fresh instead of folding into work already done.
      request.tcb->release_request(request.claim_bit());

      switch (request.type) {
         case cross_core_request::set_thread_ready:
            request.tcb->disposition = thread_disposition::none;
            // Drain inbox happens during a reschedule. No need to acknowledge the hint
            (void)set_thread_ready(*request.tcb);
            break;

         case cross_core_request::recompute_priority:
            // Value-free doorbell: re-derive from current truth. The id check
            // filters TCB recycling, every other form of staleness degrades
            // to a redundant recompute inside the walk itself.
            if (request.tcb->id == request.expected_thread_id) {
               thread_action::recompute_thread_priority(*request.tcb, request.expected_thread_id);
            }
            break;
      }
   }
}

// Cross-core safe posting API
bool scheduler::post_to_inbox(cross_core_request request) noexcept
{
   // Many-producer safe.
   //
   // Fold into an entry that is already queued and not yet serviced, rather
   // than queueing a second one saying the same thing. Claiming BEFORE the push
   // is what makes a set bit mean "a service is still coming".
   if (!request.tcb->claim_request(request.claim_bit())) return true;

   if (!inbox.push(request)) {
      // Nothing was queued, so the claim must not survive. Leaving it set would
      // fold every later request into an entry that does not exist, turning a
      // transient full into permanent silence for this target.
      request.tcb->release_request(request.claim_bit());
      return false; // Full
   }

   bool expected = false;
   if (inbox_poke_pending.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      cyros_port_send_reschedule_ipi(core_id);
   }
   return true;
}

/**
 * @brief Select the next runnable thread for this core and switch to it.
 *
 * Sole arbiter of contested transitions. Runs on the owning core in the
 * current thread's context and reconciles that thread's own wish (its
 * disposition) against any wake that raced it.
 *
 * Design: state (position) and disposition (intent to block) are orthogonal.
 * A thread authors its own disposition, but this is the ONLY place committed
 * becomes blocked, so the block decision has a single arbiter even though the
 * wish is raised from thread context.
 *
 * Policy: drain_inbox() runs first and is the reconciler. A wake clears its
 * target's disposition as it readies it, so a wake landing on a thread that
 * already committed to blocking revokes that commit here. The wake wins and
 * the thread stays runnable rather than parking on a stale decision. A
 * rotation and the pick must therefore preserve a prepared disposition, so a
 * waiter preempted mid-wait comes back still intending to block and cannot be
 * stranded.
 *
 * Dispatch on the running thread:
 *      committed          -> park, not re-enqueued
 *      none / prepared    -> rotate, prepared preserved
 *      ready              -> already readied by drain, not re-enqueued
 *      terminated         -> exiting, not re-enqueued
 *      blocked / created  -> illegal on entry (asserted)
 *
 * Entry contract: called only by the owning core, current_thread non-null
 * and not enqueued. A blocked or created thread is never the running thread.
 */
void scheduler::reschedule() noexcept
{
   CYROS_ASSERT(current_thread);
   CYROS_ASSERT(!current_thread->is_enqueued());

   drain_inbox();

   thread_control_block* previous_thread = current_thread;
   thread_control_block* next_thread     = nullptr;

   {
      // The retire-and-pick must observe and mutate the matrix atomically.
      // A pick made stale by an ISR-triggered wake_x is repaired by the
      // ISR's own pend tail-chaining a corrective reschedule.
      this_core::critical_guard guard;

      switch (previous_thread->state) {
         case thread_state::running:
            if (previous_thread->disposition == thread_disposition::committed) {
               set_thread_blocked(*previous_thread);
            } else {
               (void)set_thread_ready(*previous_thread);
            }
            break;

         case thread_state::terminated:
         case thread_state::ready:
            break;

         case thread_state::blocked:
         case thread_state::created:
            CYROS_ASSERT1(false, previous_thread->state); // Illegal thread state
            break;
      }

      next_thread = ready_matrix.pop_best_thread();
      if (!next_thread) next_thread = idle_thread;

      set_thread_running(*next_thread);
   }

   cyros_port_switch(previous_thread->context(), next_thread->context());
}

void scheduler::reset()
{
   // At shutdown the inbox may still hold stale wake requests (e.g. a signaller
   // can post wakes faster than the target drains them, and the target may
   // terminate with surplus wakes still queued). But a wake to a
   // terminated thread is a no-op. But we don't want to miss any pending work
   // to a thread that is still live and blocked.
   cross_core_request request;
   while (inbox.pop(request)) {
      // A claim never outlives the entry that holds it, on this path too.
      request.tcb->release_request(request.claim_bit());
      CYROS_ASSERT_OP(request.tcb->state, ==, thread_state::terminated);
   }
   CYROS_ASSERT(ready_matrix.empty()); // Cannot reset whilst threads still in the queue

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   inbox_poke_pending.store(false, std::memory_order_relaxed);
   current_thread = nullptr;
}

}  // namespace cyros