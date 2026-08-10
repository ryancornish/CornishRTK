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

/**
 * @brief Global state of the kernel
 *
 * All globals required in the kernel layer must live within here.
 */
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
 * @brief Load-balancing heuristic for registering threads.
 *
 * If a thread's affinity includes multiple cores, the thread will
 * be pinned to the core with the fewest threads pinned to it
 * at the time of registration. The lowest-indexed core is chosen in a tie.
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


/* How far a transitive donation is followed before the answer is taken to be
 * maximally urgent.
 *
 * Real chains are short: the deepest the suite builds is 6, and the flagship PI
 * test is 2. The budget is not a performance knob, it is what stops a wait-for
 * cycle (a deadlocked application) from being walked forever. Exhausting it
 * yields 0, which over-boosts, which is the safe direction. */
inline constexpr unsigned max_inheritance_depth = 8;

/**
 * @brief min(base, best waiter of every held PI resource).
 *
 * The definition of effective priority, folded from current truth.
 *
 * Evaluable by any core at any time and with no pi_lock: held_slots is a bounded
 * array of atomics and the queue tops are maintained atomics for exactly that.
 * The one restriction is on queue locks, see thread_action::urgency_at.
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
      floor = std::min(waitable_access::queue_top(*held, depth), floor);
   }
   return floor;
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
   return donated_floor(tcb, max_inheritance_depth);
}

std::uint8_t urgency_at(thread_control_block const& tcb, unsigned const depth) noexcept
{
   return donated_floor(tcb, depth);
}

void request_repick(thread_control_block& tcb)
{
   if (tcb.pinned_core == cyros_port_get_core_id()) {
      cyros_port_pend_reschedule();
   } else {
      cyros_port_send_reschedule_ipi(tcb.pinned_core);
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
   return scheduler_for_this_core().current_thread_urgency();
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