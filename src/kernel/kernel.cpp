#include <cyros/kernel/kernel.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "scheduler.hpp"
#include "thread_action.hpp"
#include "threading_subsystem.hpp"
#include "waitable_utilities.hpp"

#include <bit>
#include <atomic>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>

namespace cyros
{

namespace
{

struct kernel_state
{
   spinlock lock;
   std::array<scheduler, config::cores> schedulers;
   std::atomic<bool> initialised{false};
   std::atomic<bool> running{false};
   std::atomic<std::uint32_t> active_threads{0};
   std::atomic<std::uint32_t> thread_id_generator{1};

   // Compile-time construct the scheduler list with incrementing core id's.
   template<std::size_t... is>
   constexpr explicit kernel_state(std::index_sequence<is...>) noexcept : schedulers{ scheduler{is}... } {}
   constexpr kernel_state() noexcept : kernel_state(std::make_index_sequence<config::cores>{}) {}
} constinit k;

// Use this to examine how much memory the kernel uses.
[[maybe_unused]] constexpr auto kernel_memory = sizeof(k);

[[nodiscard, gnu::pure]]
scheduler& scheduler_for_core(std::uint32_t core_id)
{
   return k.schedulers.at(core_id);
}

[[nodiscard, gnu::pure]]
scheduler& scheduler_for_this_core()
{
   return k.schedulers[cyros_port_get_core_id()];
}

/**
 * @brief Load-balancing primer for registering threads
 */
void pin_thread_to_core(thread_control_block& tcb) noexcept
{
   std::uint32_t best_core = std::numeric_limits<std::uint32_t>::max();
   std::uint32_t best_load = std::numeric_limits<std::uint32_t>::max();

   bool found = false;
   for (std::uint32_t core = 0; core < config::cores; ++core) {
      if (!tcb.affinity.allows(core)) continue;

      auto const load = scheduler_for_core(core).pinned_thread_count();
      if (load < best_load) {
         best_load = load;
         best_core = core;
         found = true;
      }
   }
   CYROS_ASSERT(found); // thread affinity mask allows no cores

   scheduler_for_core(best_core).pin_thread(tcb);
}

/**
 * @brief Post a priority-recompute doorbell to a thread's owning core.
 *
 * Value-free by design: the request tells the owning core to re-derive the
 * thread's effective priority from current truth, it never carries a
 * priority. Posting to the calling core itself is legal and used as the
 * overflow path of a deep local boost chain.
 */
void post_priority_recompute(thread_control_block& tcb, thread::id const expected_id)
{
   // expected_id is deliberately not carried across: the claim lives on the TCB
   // and thread construction is a placement new, so a recycled TCB comes back
   // with pending_requests zeroed and cannot present a stale request.
   (void)expected_id;
   scheduler_for_core(tcb.pinned_core).post_intake(tcb, thread_request::recompute_priority);
}

/**
 * @brief Pending targets of a priority-inheritance chain walk.
 *
 * Bounded because the walk can run on the shared interceptor stack via
 * drain_intake: local hops beyond the capacity continue by self-posted
 * doorbell instead of growing the walk, which push() folds away from the
 * traversal.
 */
struct priority_chain
{
   static constexpr auto depth = 8u;
   struct target
   {
      thread_control_block* tcb;
      thread::id expected_id;
   };

   std::array<target, depth> targets{};
   std::size_t pending{0};

   void push(thread_control_block& next, thread::id const next_expected_id) noexcept
   {
      if (pending < targets.size()) {
         targets[pending++] = { .tcb = &next, .expected_id = next_expected_id };
      } else {
         post_priority_recompute(next, next_expected_id);
      }
   }
};

/**
 * @brief min(base, best waiter of every held PI resource).
 *
 * The definition of effective priority, folded from current truth.
 *
 * NO LOCK REQUIRED. held_slots is a bounded array of atomics and the queue tops
 * are maintained atomics, both so this can be evaluated by any core at any time.
 * The recompute walk happens to hold the target's pi_lock when it calls this,
 * but that is the walk's business (it also mutates), not a precondition here.
 */
[[nodiscard]] std::uint8_t donated_floor(thread_control_block const& target) noexcept
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
      floor = std::min(waitable_access::queue_top(*held), floor);
   }
   return floor;
}

/**
 * @brief Re-order a blocked thread's armed nodes after its priority changed.
 *
 * The ordered position arm() gave each node is now stale. Where a re-slot
 * changes a queue's BEST waiter, the holder of that queue's resource
 * inherits differently, so it is chained for its own recompute, processed
 * after this target's pi_lock is released. Caller holds the target's
 * pi_lock, which keeps active_waits stable.
 */
void reslot_blocked_waits(thread_control_block& target, priority_chain& chain) noexcept
{
   if (target.state != thread_state::blocked || target.active_waits == nullptr) {
      return;
   }
   for (auto& node : *target.active_waits) {
      if (node.source == nullptr) continue;
      if (!waitable_access::reslot(*node.source, node)) continue;

      thread::id chase_id = 0;
      if (auto* next = waitable_access::donation_target(*node.source, chase_id)) {
         chain.push(*next, chase_id);
      }
   }
}


// Registered as isr handler for preemptive scheduling
void reschedule_this_core()
{
   auto& scheduler = scheduler_for_this_core();

   scheduler.reschedule();
}

void core_entry()
{
   auto& scheduler = scheduler_for_this_core();

   scheduler.start();
}

} // namespace

namespace thread_action
{

thread_control_block& get_current_thread_on_this_core()
{
   auto& scheduler = scheduler_for_this_core();

   return scheduler.get_current_thread();
}

void register_thread(thread_control_block& tcb)
{
   CYROS_ASSERT_OP(tcb.state, ==, thread_state::created);
   CYROS_ASSERT(!tcb.is_enqueued());

   tcb.id = k.thread_id_generator.fetch_add(1, std::memory_order_relaxed);
   k.active_threads.fetch_add(1, std::memory_order_seq_cst);

   {
      spinlock_guard guard(k.lock);
      pin_thread_to_core(tcb);
   }
   auto& scheduler = scheduler_for_core(tcb.pinned_core);

   // If cores are not running yet, enqueue directly (even for remote cores)
   if (!k.running.load(std::memory_order_acquire)) {
      (void)scheduler.set_thread_ready(tcb);
      return;
   }

   if (ready_thread(tcb) == schedule_hint::warranted) {
      // Weak request: Registering the thread is not itself blocking, it is
      // only flagging that a higher-priority thread became ready.
      cyros_port_pend_reschedule();
   }
}

schedule_hint ready_thread(thread_control_block& tcb)
{
   // Fast path: if the thread is already terminated. Can happen with stale remote-ready-requests
   // A thread that terminates AFTER this check is handled by the scheduler-level guard when the request is drained.
   if (tcb.state == thread_state::terminated) {
      return schedule_hint::unwarranted;
   }

   auto& scheduler = scheduler_for_core(tcb.pinned_core);

   auto const this_core = cyros_port_get_core_id();
   if (this_core != tcb.pinned_core) {
      scheduler.post_intake(tcb, thread_request::make_ready);
      return schedule_hint::unwarranted;
   }

   return scheduler.set_thread_ready(tcb);
}

std::uint8_t urgency(thread_control_block const& tcb) noexcept
{
   return donated_floor(tcb);
}

void track_holder(thread_control_block& tcb)
{
   scheduler_for_core(tcb.pinned_core).link_holder(tcb);
}

void untrack_holder(thread_control_block& tcb)
{
   scheduler_for_core(tcb.pinned_core).unlink_holder(tcb);
}

void recompute_thread_priority(thread_control_block& tcb, thread::id const expected_id)
{
   priority_chain chain;
   chain.push(tcb, expected_id);

   auto const this_core_id = cyros_port_get_core_id();
   bool reschedule_warranted = false;

   while (chain.pending > 0) {
      auto const target = chain.targets[--chain.pending];

      if (target.tcb->id != target.expected_id) continue;          // TCB recycled, drop
      if (target.tcb->state == thread_state::terminated) continue; // Stale, drop

      if (target.tcb->pinned_core != this_core_id) {
         post_priority_recompute(*target.tcb, target.expected_id);
         continue;
      }

      // pi_lock holds the held list and active_waits stable, and (as an
      // interrupt-masking spinlock) makes the matrix surgery inside
      // reprioritise_thread atomic against ISRs as well as thread switches.
      spinlock_guard pi_guard(target.tcb->pi_lock);

      std::uint8_t const new_effective = donated_floor(*target.tcb);
      if (new_effective == target.tcb->priority()) continue;

      auto const hint = scheduler_for_this_core().reprioritise_thread(*target.tcb, new_effective);
      if (hint == schedule_hint::warranted) {
         reschedule_warranted = true;
      }

      reslot_blocked_waits(*target.tcb, chain);
   }

   if (reschedule_warranted) {
      cyros_port_pend_reschedule();
   }
}

} // namespace thread_action

void thread_launcher(void* tcb_ptr)
{
   auto* tcb = static_cast<thread_control_block*>(tcb_ptr);

   cyros_port_set_tls_pointer(tcb); // for now point tls to the tcb, but in future, tls sits just after tcb

   tcb->entry();

   auto& scheduler = scheduler_for_this_core();

   // Idle threads are outside the teardown bookkeeping and return normally
   if (tcb->id == scheduler::idle_thread_id) {
      scheduler.set_thread_terminated(*tcb);
      return;
   }

   // Teardown of the user thread must not be interrupted.
   // Bookkeeping and port exit mechanics must be made
   // atomically. It is up to the port exit routine to
   // return us to the reschedule routine.
   auto token = cyros_port_preempt_disable();

   scheduler.set_thread_terminated(*tcb);
   CYROS_ASSERT(k.active_threads.fetch_sub(1, std::memory_order_seq_cst) != 0);

   cyros_port_thread_exit(token);
}

void idle_task()
{
   // We may have received a request whilst bootstrapping (if idle_thread was first picked)
   if (scheduler_for_this_core().intake_pending()) {
      this_thread::yield();
   }

   while (k.running.load(std::memory_order::relaxed)) {
      // Re-check BEFORE sleeping. cyros_port_idle() blocks until signalled, so
      // a notification lost between a producer's push and this point would
      // otherwise park the core with work outstanding, which is a hang rather
      // than a delay. The intake is the truth and the IPI only a hint, so
      // looking again here is what makes losing the hint survivable.
      if (scheduler_for_this_core().intake_pending()) {
         this_thread::yield();
         continue;
      }

      cyros_port_idle();

      this_thread::yield();
   }
}

namespace kernel
{

void initialise() noexcept
{
   CYROS_ASSERT(!k.initialised); // Cannot invoke kernel::initialise twice (without finalising down in between)

   cyros_port_init(reschedule_this_core);

   for (auto& scheduler : k.schedulers) {
      scheduler.init_idle_thread();
   }
   k.initialised.store(true, std::memory_order_relaxed);
}

void start() noexcept
{
   CYROS_ASSERT(k.initialised); // kernel::initialise() must be called first
   CYROS_ASSERT(k.active_threads > 0); // Starting the kernel with no registered threads is not allowed

   k.running.store(true, std::memory_order_release);

   cyros_port_start_cores(config::cores, core_entry);
}

void finalise() noexcept
{
   CYROS_ASSERT(k.initialised);

   k.running.store(false, std::memory_order_relaxed);
   k.thread_id_generator.store(1, std::memory_order_relaxed);
   k.active_threads.store(0, std::memory_order_relaxed);;

   for (auto& scheduler : k.schedulers) {
      scheduler.reset();
   }
   k.initialised.store(false, std::memory_order_relaxed);
}

std::uint32_t core_count() noexcept
{
   return config::cores;
}

std::uint32_t active_threads() noexcept
{
   return k.active_threads.load(std::memory_order_relaxed);
}

} // namespace kernel

namespace this_core
{

[[nodiscard]] std::uint32_t id() noexcept
{
   return cyros_port_get_core_id();
}

void pend_reschedule() noexcept
{
   cyros_port_pend_reschedule();
}

preemption_token disable_preemption() noexcept
{
   return { .v = cyros_port_preempt_disable() };
}

void enable_preemption(preemption_token token) noexcept
{
   cyros_port_preempt_enable(token.v);
}

critical_token enter_critical() noexcept
{
   return { .v = cyros_port_irq_save() };
}

void exit_critical(critical_token token) noexcept
{
   cyros_port_irq_restore(token.v);
}

} // namespace this_core

namespace this_thread
{

[[nodiscard]] thread::id id()
{
   return scheduler_for_this_core().current_thread_id();
}

[[nodiscard]] thread::priority priority()
{
   return scheduler_for_this_core().current_thread_priority();
}

[[noreturn]] void thread_exit()
{
   // todo
   CYROS_PORT_UNREACHABLE();
}

void yield()
{
   // Strong request: an explicit yield deliberately gives up the CPU and
   // relies on the reschedule round-trip having completed on return.
   cyros_port_thread_yield();
}

[[nodiscard]] std::size_t wait_on_any(std::span<waitable_ref> waitables) noexcept
{
   CYROS_ASSERT(!waitables.empty());
   CYROS_ASSERT_OP(waitables.size(), <=, config::max_wait_nodes);

   auto& tcb = thread_action::get_current_thread_on_this_core();
   wait_node_vector nodes(waitables.size(), tcb);

   active_wait_registration const registration(tcb, &nodes);

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
            if (waitable.wait_condition(*tcb.public_thread_handle)) {
               tcb.disposition = thread_disposition::none;
               chosen = i;
               break;
            }
            ++i;
         }

         if (!chosen) {
            auto token = cyros_port_preempt_disable();
            if (tcb.disposition == thread_disposition::prepared) {
               // Condition unsatisfied AND no wake came after arming, block until woken
               tcb.disposition = thread_disposition::committed;
               cyros_port_pend_reschedule(); // Delayed until preempt_enable()
            }
            cyros_port_preempt_enable(token);
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

}  // namespace this_thread

}  // namespace cyros