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

   // Idempotent: if already admitted, this is a redundant wake/admit (e.g.
   // two signallers raced, or a stale wake from a prior round). The thread
   // is already going to run.
   if (tcb.is_enqueued() || tcb.is_listed_holder()) {
      CYROS_ASSERT(tcb.state == thread_state::ready);
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

uint8_t scheduler::current_thread_urgency() const noexcept
{
   return current_thread ? thread_action::urgency(*current_thread) : 0;
}

thread_control_block* scheduler::pick_next() noexcept
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
   // Ties go to the matrix, preserving the FIFO fairness round robin relies on.
   if (best_base >= 0 && static_cast<std::uint8_t>(best_base) <= best_holder_urgency) {
      return ready_matrix.pop_best_thread();
   }

   unlink_holder(*best_holder);
   return best_holder;
}

void scheduler::service_intake(thread_control_block& tcb, std::uint8_t bits) noexcept
{
   if (bits & thread_control_block::request_bit(thread_request::make_ready)) {
      tcb.disposition = thread_disposition::none;
      // Runs during a reschedule, so there is no hint to acknowledge.
      (void)set_thread_ready(tcb);
   }

}

void scheduler::post_intake(thread_control_block& tcb, thread_request request) noexcept
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

void scheduler::link_holder(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (tcb.is_listed_holder()) return; // already tracked

   tcb.holder_next = holders_head;
   holders_head = &tcb;
}

void scheduler::unlink_holder(thread_control_block& tcb) noexcept
{
   CYROS_ASSERT_OP(tcb.pinned_core, ==, core_id);
   if (!tcb.is_listed_holder()) return; // not tracked

   // Singly linked and removed by search. The list is holders-on-this-core,
   // measured at about one, so the walk is not worth a back pointer, and TCB
   // bytes are the scarcer resource.
   auto** slot = &holders_head;
   while (*slot != nullptr && *slot != &tcb) {
      slot = &(*slot)->holder_next;
   }
   CYROS_ASSERT(*slot == &tcb); // holder missing from its own core's list
   *slot = tcb.holder_next;
   tcb.holder_next = &tcb; // restore the not-linked sentinel
}

bool scheduler::push_intake(thread_control_block& tcb) noexcept
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
static thread_control_block* await_intake_link(thread_control_block* node) noexcept
{
   thread_control_block* next = nullptr;
   while ((next = node->intake_next.load(std::memory_order_acquire)) == node) {
      cyros_port_cpu_relax();
   }
   return next;
}

void scheduler::drain_intake() noexcept
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
 * Policy: drain_intake() runs first and is the reconciler. A wake clears its
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

   drain_intake();

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

      next_thread = pick_next();
      if (!next_thread) next_thread = idle_thread;

      set_thread_running(*next_thread);
   }

   cyros_port_switch(previous_thread->context(), next_thread->context());
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

   pinned_thread_counter.store(0, std::memory_order_relaxed);
   current_thread = nullptr;
}

}  // namespace cyros