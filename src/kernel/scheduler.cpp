#include "scheduler.hpp"

#include "thread_action.hpp"

#include <cyros/kernel/core.hpp>

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

void scheduler::start()
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

schedule_hint scheduler::set_thread_ready(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);

   // Interrupt-masked because this is the choke point every wake funnels
   // through, including ISR wakes: the matrix splice below must be atomic
   // against both a cross-core reschedule IPI landing mid-splice (a latent
   // race even in thread context) and an ISR wake re-entering it.
   this_core::critical_guard guard;

   // Idempotent: a remote core might have sent us a late request to ready
   // this thread, and we have already terminated it since (or are terminating). No-op.
   if (tcb.state == thread_state::terminated || tcb.disposition == thread_disposition::terminating) {
      return schedule_hint::unwarranted;
   }

   // Idempotent: if already admitted, this is a redundant wake/admit (e.g.
   // two signallers raced, or a stale wake from a prior round). The thread
   // is already going to run.
   if (tcb.is_enqueued() || tcb.is_listed_holder()) {
      CYROS_ASSERT(tcb.state == thread_state::ready);

      // Re-route before returning. Ownership can arrive AFTER admission:
      // hand_over files a handed-over resource into the new owner's slots from
      // the releasing core, and a thread that was already sitting in the ready
      // matrix at that moment is a holder filed where the urgency fold never
      // looks, which is unbounded inversion for whoever parks behind the
      // resource it now owns. Every such handover is followed by a redundant
      // admission of the new owner, which is what makes this the one place the
      // routing can be re-derived without a new message. The reverse migration
      // cannot be needed: only a running thread can release, so a READY
      // holder cannot stop being one.
      if (tcb.is_enqueued() && !tcb.holds_nothing()) {
         ready_matrix.remove_thread(tcb);
         link_holder(tcb);
         if (thread_action::urgency(tcb) < current_thread_urgency()) {
            return schedule_hint::warranted;
         }
      }
      return schedule_hint::unwarranted;
   }

   tcb.state = thread_state::ready;

   // Idle thread does not belong in the ready_matrix,
   // but DOES follow state transition semantics
   if (&tcb == idle_thread) {
      return schedule_hint::unwarranted;
   }

   // A thread holding a pi_waitable can be more urgent than its base priority,
   // so it cannot be filed under a fixed key. It goes on the holder list, which
   // the pick folds urgency over. Everyone else is filed at base_priority, which
   // never changes, so nothing ever re-keys the matrix.
   if (tcb.holds_nothing()) {
      ready_matrix.enqueue_thread(tcb);
   } else {
      link_holder(tcb);
   }

   if (thread_action::urgency(tcb) < current_thread_urgency()) {
      return schedule_hint::warranted;
   }
   return schedule_hint::unwarranted;
}

void scheduler::set_thread_running(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::ready);

   tcb.state = thread_state::running;
   current_thread = &tcb;
}

void scheduler::set_thread_blocked(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
   CYROS_ASSERT_OP(tcb.disposition, ==, thread_disposition::committed);
   CYROS_ASSERT(!tcb.is_enqueued());

   tcb.disposition = thread_disposition::none;
   tcb.state = thread_state::blocked;
}

void scheduler::set_thread_terminated(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::running);
   CYROS_ASSERT(tcb.disposition == thread_disposition::terminating);
   CYROS_ASSERT(&tcb != idle_thread); // A core always needs somewhere to go
   CYROS_ASSERT(!tcb.is_enqueued());
   CYROS_ASSERT(tcb.holds_nothing()); // Thread cannot own a pi_waitable on termination

   tcb.state = thread_state::terminated;

   thread_action::unregister_thread(tcb);

   // Signal any and all joiners waiting on this thread
   tcb.termination.terminate();

   cyros_port_retire_context(tcb.context());
}

uint8_t scheduler::current_thread_urgency() const
{
   return current_thread ? thread_action::urgency(*current_thread) : 0;
}

thread_control_block* scheduler::pick_next()
{
   // The matrix is keyed on base_priority, which no boost can change, so its
   // head is the most urgent NON-holder by construction.
   auto const best_base = ready_matrix.best_priority();

   // Holders are the only threads whose urgency can beat their base priority,
   // and they are rare: measured at about 1 percent of picks in the one test
   // that exercises mutexes, and zero everywhere else. So this loop almost
   // always runs zero times and the pick is exactly what it always was.
   thread_control_block* best_holder = nullptr;
   std::uint8_t best_holder_urgency = 0xFF;
   for (auto* h = holders_head; h != nullptr; h = h->holder_next) {
      auto const u = thread_action::urgency(*h);
      if (u < best_holder_urgency) {
         best_holder_urgency = u;
         best_holder = h;
      }
   }

   if (best_holder == nullptr) {
      return ready_matrix.pop_best_thread();
   }
   if (best_base >= 0) {
      auto const base = static_cast<std::uint8_t>(best_base);
      if (base < best_holder_urgency) {
         return ready_matrix.pop_best_thread();
      }
      if (base == best_holder_urgency) {
         /* No arrival order spans the two structures, so alternate. Giving the
          * tie to either side outright starves the other outright: the loser is
          * re-examined on the next pick and loses identically, because being
          * passed over does not change its position. See tie_rotor. */
         auto const bit = std::uint32_t{1} << base;
         tie_rotor ^= bit;
         if ((tie_rotor & bit) == 0) {
            return ready_matrix.pop_best_thread();
         }
      }
   }

   unlink_holder(*best_holder);
   return best_holder;
}

void scheduler::service_intake(thread_control_block& tcb, std::uint8_t bits)
{
   if (bits & thread_control_block::request_bit(thread_request::make_ready)) {
      if (tcb.disposition != thread_disposition::terminating) {
         tcb.disposition = thread_disposition::none;
      }
      // Runs during a reschedule, so there is no hint to acknowledge.
      (void)set_thread_ready(tcb);
   }

}

void scheduler::post_intake(thread_control_block& tcb, thread_request request)
{
   auto const bit = thread_control_block::request_bit(request);

   // One atomic answers both questions. A previously-zero field means this TCB
   // is not on the intake and we must put it there. Any non-zero value means it
   // already is, and since these requests carry truth rather than values, the
   // entry already queued discharges this intent too.
   auto const old = tcb.pending_requests.fetch_or(bit, std::memory_order_acq_rel);
   if (old != 0) return;

   if (push_intake(tcb)) {
      // Empty-to-non-empty, so nobody else has poked this core for this batch.
      cyros_port_send_reschedule_ipi(core_id);
   }
}

void scheduler::link_holder(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (tcb.is_listed_holder()) return; // already tracked

   // Appended, not pushed. pick_next keeps the first holder at the best
   // urgency, so head insertion would give every tie to the most recently
   // linked, which is the thread that just ran. See scheduler::holders_head.
   tcb.holder_next = nullptr;
   if (holders_tail == nullptr) {
      holders_head = &tcb;
   } else {
      holders_tail->holder_next = &tcb;
   }
   holders_tail = &tcb;
}

void scheduler::unlink_holder(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (!tcb.is_listed_holder()) return; // not tracked

   // Singly linked and removed by search. The list is holders-on-this-core,
   // measured at about one, so the search is not worth a back pointer in the TCB,
   // and TCB bytes are the scarcer resource. The predecessor is tracked here
   // rather than through a pointer-to-pointer walk only because the tail needs
   // repairing when the last element goes.
   thread_control_block* previous = nullptr;
   auto* current = holders_head;
   while (current != nullptr && current != &tcb) {
      previous = current;
      current  = current->holder_next;
   }
   CYROS_ASSERT(current == &tcb); // holder missing from its own core's list

   if (previous == nullptr) {
      holders_head = tcb.holder_next;
   } else {
      previous->holder_next = tcb.holder_next;
   }
   if (holders_tail == &tcb) {
      holders_tail = previous;
   }
   tcb.holder_next = &tcb; // restore the not-linked sentinel
}

bool scheduler::push_intake(thread_control_block& tcb)
{
   // The self-pointer says "position published, link not yet". It must be in
   // place BEFORE the exchange makes the node reachable. nullptr cannot serve as
   // this marker: the bottom of the chain legitimately ends in nullptr, and a
   // consumer reading it there would stop early and strand everything beneath.
   tcb.intake_next.store(&tcb, std::memory_order_relaxed);

   // Interrupts masked across the exchange and the link store. This is what
   // bounds the consumer's wait in await_intake_link to two instructions, and
   // it is the entire reason an exchange is acceptable here instead of a CAS
   // retry loop. Preempt-grade would NOT do: the timer ISR would still fire and
   // can itself wake a thread, so a same-core ISR could run a whole push while
   // this node is still showing the sentinel.
   //
   // enter_critical is depth-based, so calling this from an ISR that already
   // masked (a future chrono alarm doing wake_one) simply nests.
   this_core::critical_guard guard;

   auto* prev = intake_head.exchange(&tcb, std::memory_order_acq_rel);

   tcb.intake_next.store(prev, std::memory_order_release);

   // Empty-to-non-empty is decided by the same atomic that enqueued, so unlike a
   // separate poke flag it cannot go stale against the queue it describes.
   return prev == nullptr;
}

/**
 * @brief Wait for a producer to publish a node's link. Bounded, see push_intake.
 */
static thread_control_block* await_intake_link(thread_control_block* node)
{
   thread_control_block* next = nullptr;
   while ((next = node->intake_next.load(std::memory_order_acquire)) == node) {
      cyros_port_cpu_relax();
   }
   return next;
}

void scheduler::drain_intake()
{
   auto* node = intake_head.exchange(nullptr, std::memory_order_acq_rel);
   if (node == nullptr) return;

   // The stack is LIFO and thread_ready_queue is FIFO within a priority, which
   // round robin's fairness depends on, so restore arrival order before
   // servicing. O(taken), same as the walk that follows.
   thread_control_block* fifo = nullptr;
   while (node != nullptr) {
      auto* next = await_intake_link(node);
      node->intake_next.store(fifo, std::memory_order_relaxed);
      fifo = node;
      node = next;
   }

   while (fifo != nullptr) {
      auto* next = fifo->intake_next.load(std::memory_order_relaxed);
      fifo->intake_next.store(nullptr, std::memory_order_relaxed);

      // Release the claims BEFORE servicing, so a request raised while we are
      // servicing this one queues fresh instead of folding into work already
      // done. acq_rel because taking all bits at once is a read-modify-write.
      auto const bits = fifo->pending_requests.exchange(0, std::memory_order_acq_rel);

      service_intake(*fifo, bits);
      fifo = next;
   }
}

/**
 * @brief AKA: The 'ARBITER'. Select the next runnable thread for this core and switch to it.
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
 * Policy: drain_intake() runs first and is the reconciler. A wake clears its
 * target's disposition as it readies it, so a wake landing on a thread that
 * already committed to blocking revokes that commit here. The wake wins and
 * the thread stays runnable rather than parking on a stale decision. A
 * rotation and the pick must therefore preserve a prepared disposition, so a
 * waiter preempted mid-wait comes back still intending to block and cannot be
 * stranded. Termination is the one wish a wake may NOT revoke, and that is
 * enforced where the wake lands rather than here: see
 * thread_control_block::is_terminating.
 *
 * Dispatch on the running thread, by its disposition:
 *      terminating        -> retired, joiners woken, context handed to the port
 *      committed          -> park, not re-enqueued
 *      none / prepared    -> rotate, prepared preserved
 * and by its state:
 *      ready              -> already readied by drain, not re-enqueued
 *      terminated         -> retired by an earlier pass, not re-enqueued
 *      blocked / created  -> illegal on entry (asserted)
 *
 * Entry contract: called only by the owning core, current_thread non-null
 * and not enqueued. A blocked or created thread is never the running thread.
 */
void scheduler::reschedule()
{
   CYROS_ASSERT(current_thread);
   CYROS_ASSERT(!current_thread->is_enqueued());

   drain_intake();

   thread_control_block* previous_thread = current_thread;
   thread_control_block* next_thread     = nullptr;
   bool                  discard_prev    = false;

   {
      // The retire-and-pick must observe and mutate the matrix atomically.
      // A pick made stale by an ISR-triggered wake_x is repaired by the
      // ISR's own pend tail-chaining a corrective reschedule.
      this_core::critical_guard guard;

      switch (previous_thread->state) {
         case thread_state::running:
            if (previous_thread->disposition == thread_disposition::terminating) {
               set_thread_terminated(*previous_thread);
               discard_prev = true;
            } else if (previous_thread->disposition == thread_disposition::committed) {
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
         default:
            CYROS_ASSERT1(false, previous_thread->state); // Illegal thread state
            break;
      }

      next_thread = pick_next();
      if (!next_thread) next_thread = idle_thread;

      set_thread_running(*next_thread);
   }

   cyros_port_switch(discard_prev ? nullptr : previous_thread->context(), next_thread->context());
}

void scheduler::reset()
{
   // At shutdown the intake may still hold stale requests: a signaller can post
   // faster than the target drains, and the target may terminate with surplus
   // wakes outstanding. A request against a terminated thread is a no-op, but we
   // must not leave the chain behind.
   // The intake links live in TCBs, which are user memory that outlives the
   // kernel run, so a leftover chain would be inherited by the next lifecycle.
   // Construction is a placement new so a REUSED TCB comes back clean, but a
   // TCB that is merely still alive would not. Clear both ends here.
   auto* node = intake_head.exchange(nullptr, std::memory_order_relaxed);
   while (node != nullptr) {
      // No await_intake_link here: every core thread is joined before teardown,
      // so no push can be in flight and no link can still be unpublished.
      auto* next = node->intake_next.load(std::memory_order_relaxed);
      node->intake_next.store(nullptr, std::memory_order_relaxed);
      node->pending_requests.store(0, std::memory_order_relaxed);
      CYROS_ASSERT_OP(node->state, ==, thread_state::terminated);
      node = next;
   }
   CYROS_ASSERT(ready_matrix.empty()); // Cannot reset whilst threads still in the queue

   /* A holder leaves the list only by being picked, and a thread can only
    * release its last resource or terminate while running, i.e. already off it.
    * So an occupant here means a thread was readied as a holder and the core
    * never ran it, which is the starvation this list's FIFO order exists to
    * prevent. */
   CYROS_ASSERT(holders_head == nullptr);
   holders_tail = nullptr;
   // Not correctness, but a lifecycle should not inherit the previous one's
   // tie phase: these objects outlive kernel::finalise.
   tie_rotor = 0;

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   current_thread = nullptr;
}

}  // namespace cyros