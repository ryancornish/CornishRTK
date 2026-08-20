#ifndef CYROS_SCHEDULER_HPP
#define CYROS_SCHEDULER_HPP

#include "threading_subsystem.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cyros
{

/**
 * @brief A return-type to communicate whether the caller
 *        ought to trigger a reschedule
 */
enum class [[nodiscard]] schedule_hint
{
   unwarranted, ///< The action did not ready any better thread that currently running
   warranted,   ///< The action may have readied a better thread that currently running
};

void idle_task();

/**
 * @brief Per-core state controller
 *
 * Owns its own special thread: 'idle thread' (id: 0)
 */
class scheduler
{
private:
   std::uint32_t const core_id;
   std::atomic<uint32_t> pinned_thread_counter{0};
   thread_control_block* current_thread{nullptr};
   thread_control_block*    idle_thread{nullptr};
   alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, thread::min_stack_size> idle_stack{};

   thread_ready_matrix ready_matrix;

   /* Head of this core's intake stack. Any core pushes, only this one takes.
    *
    * The TCB is the message: what a producer wants is recorded in
    * thread_control_block::pending_requests, and this list only says WHICH TCBs
    * to look at.*/
   std::atomic<thread_control_block*> intake_head{nullptr};

   /* Threads pinned here that hold at least one base_mutex, i.e. exactly the
    * threads whose urgency can differ from their base priority. Core-local and
    * needs no atomics: see thread_control_block::holder_next.
    *
    * FIFO, appended at the tail, for the same reason every matrix level is:
    * pick_next keeps the FIRST holder at the best urgency, so head insertion
    * would hand the tie to the most recently linked. A holder is unlinked when
    * it is picked and re-linked when it is re-readied, so under head insertion
    * the thread that just ran would go straight back to the front and win
    * again, starving every other holder tied with it. */
   thread_control_block* holders_head{nullptr};
   thread_control_block* holders_tail{nullptr};

   /* Which side won the last urgency tie, one bit per priority level.
    *
    * A boosted holder is not in the matrix, so when its urgency exactly equals
    * the matrix's best base there is no arrival order to compare and the winner
    * has to be chosen by policy. Choosing a side outright starves the other
    * outright: whichever loses is never re-examined, because the losing side's
    * position does not change by being passed over. Alternating bounds the wait
    * for both at two picks per tied group.
    *
    * PER LEVEL rather than a single bool. One shared bit lets ties at two
    * different levels consume each other's turns, so a level whose ties always
    * land on the same phase starves anyway, which is the bug this is meant to
    * fix wearing a different hat.
    *
    * This costs proportionality: the split is even between the two GROUPS, so a
    * lone holder takes half against n matrix threads rather than 1/(n+1). That
    * is deliberate. Bounded delay is the property that matters at equal
    * priority, and the surplus goes to a thread inside a critical section,
    * which shortens it and shrinks the blocking time of whoever is waiting on
    * it. Same direction as the bridge-overflow over-boost in wait_queue::top.*/
   std::uint32_t tie_rotor{0};
   static_assert(config::max_priorities <= std::numeric_limits<decltype(tie_rotor)>::digits,
                 "tie_rotor cannot represent that many priorities!");

public:
   static constexpr thread::id idle_thread_id = 0; // Reserved

   constexpr explicit scheduler(std::size_t core_id) : core_id(core_id) {};

   ~scheduler() = default;
   scheduler(scheduler&&) = delete;
   scheduler(scheduler const&) = delete;
   scheduler& operator=(scheduler&&) = delete;
   scheduler& operator=(scheduler const&) = delete;

   [[nodiscard]] constexpr thread::id current_thread_id() const
   {
      return current_thread ? current_thread->id : 0;
   }

   /**
    * @brief Urgency of the running thread, folded from truth.
    */
   [[nodiscard]] uint8_t current_thread_urgency() const;

   /**
    * @brief Threads pinned here. Used by pin_thread_to_core to load balance.
    */
   [[nodiscard]] uint32_t pinned_thread_count() const
   {
      return pinned_thread_counter.load(std::memory_order_relaxed);
   }

   [[nodiscard]] constexpr thread_control_block& get_current_thread() const
   {
      CYROS_ASSERT(current_thread != nullptr); // Not invocable from non-thread context

      return *current_thread;
   }

   void pin_thread(thread_control_block& tcb);

   void init_idle_thread();

   void start();

   schedule_hint set_thread_ready(thread_control_block& tcb);

   void set_thread_running(thread_control_block& tcb);

   void set_thread_blocked(thread_control_block& tcb);

   void set_thread_terminated(thread_control_block& tcb);

   /**
    * @brief Choose the most urgent runnable thread on this core.
    *
    * Best of the matrix head (most urgent non-holder, by base priority) and the
    * most urgent holder, whose urgency is folded from truth rather than read
    * from a cache. Returns nullptr when nothing is runnable.
    */
   thread_control_block* pick_next();

   /**
    * @brief Push a TCB onto this core's intake stack. Any core may call.
    *
    * @return true when the intake was EMPTY before this push, meaning the
    *         caller owns sending the IPI. That decision comes from the same
    *         atomic that enqueues, so there is no separate "is a drain already
    *         coming" flag to go stale against it.
    *
    * Caller must already hold the claim in tcb.pending_requests, so a TCB is
    * on the stack at most once.
    */
   [[nodiscard]] bool push_intake(thread_control_block& tcb);

   /**
    * @brief Ask this core to service @p type for @p tcb. Any core may call.
    *
    * Claims, enqueues and notifies. There is no failure mode: the TCB is the
    * message and it always has room for itself, so nothing can overflow and
    * there is no full-queue case needing a correct response.
    */
   void post_intake(thread_control_block& tcb, thread_request request);

   /**
    * @brief Act on every request bit set for @p tcb. Owning core only.
    */
   void service_intake(thread_control_block& tcb, std::uint8_t bits);

   /**
    * @brief Take the whole intake chain and service every request on it.
    *
    * Single consumer: only the owning core may call this.
    */
   void drain_intake();

   /**
    * @brief Append a ready thread that holds something to the holder list.
    *
    * Called from set_thread_ready, not from acquisition: a thread that takes a
    * resource is running, so it joins the list at its next re-ready. Idempotent.
    */
   void link_holder(thread_control_block& tcb);

   /**
    * @brief Remove a holder from the list because the pick chose it.
    *
    * The counterpart to link_holder, and the only way out of the list. A thread
    * that releases its last resource is running, so it is already off it.
    */
   void unlink_holder(thread_control_block& tcb);

   /**
    * @brief True when this core has intake work outstanding. Advisory.
    *
    * Cheap enough for the idle loop to check before sleeping, which is what
    * keeps a lost IPI to a scheduling round instead of a hang.
    */
   [[nodiscard]] bool intake_pending() const
   {
      // Relaxed: this only decides whether to look, never what is true.
      return intake_head.load(std::memory_order_relaxed) != nullptr;
   }

   /**
   * @brief Select the next runnable thread for this core and switch to it.
   *
   * Pick the highest-priority ready thread for this core and context-switch
   * to it, parking or re-enqueuing the outgoing thread as its state and
   * disposition dictate. Runs on the owning core in the current thread's
   * context, driven by a yield, a wake, or a preemption IPI. See the .cpp
   * for the full transition policy.
   */
   void reschedule();

   void reset();
};

/**
 * @brief The scheduler for the calling core.
 *
 * DEFINED IN kernel.cpp, not scheduler.cpp. The scheduler array is a member of
 * the kernel_state singleton, which lives in kernel.cpp's anonymous namespace so
 * that no other translation unit can reach the instance. This is the declared
 * door to it, and the only one: reaching ANOTHER core's scheduler stays private
 * to kernel.cpp behind global_ready_thread, because that is where
 * the local-versus-remote decision and the cross-core intake protocol live.
 *
 * Safe by construction in a way scheduler_for_core(id) is not. A core can only
 * ever obtain its own, and every scheduler method that mutates asserts its
 * pinned-core precondition anyway.
 *
 * [[gnu::pure]] is sound only because threads never migrate cores, so a core's
 * scheduler is a fixed answer for the life of the call. If migration is ever
 * implemented, revisit this attribute before anything else.
 */
[[nodiscard, gnu::pure]] scheduler& scheduler_for_this_core();

/**
 * @brief Make a thread runnable on its pinned core (cross-core-safe).
 *
 * Transitions @p tcb to the ready state and enqueues it on the ready queue of
 * its pinned core. If the thread belongs to another core, a cross-core request
 * is posted so that the owning scheduler performs the enqueue.
 *
 * The cross-core sibling of scheduler::set_thread_ready, and the reason
 * scheduler_for_core(id) stays private: this is where the local-versus-remote
 * decision and the intake protocol live, so nothing else needs to reach a
 * remote scheduler. DEFINED IN kernel.cpp for that access.
 *
 * @param tcb thread control block of the thread to make runnable.
 * @return schedule_hint::warranted if the thread was queued on the current core
 *         and has higher priority than the running thread, indicating the caller
 *         should request a local reschedule.
 */
schedule_hint global_ready_thread(thread_control_block& tcb);

/* ---------------------------------------------------------------------------
 * A thread's relationship to scheduling: what it is worth, when it parks, and
 * when a core should think again. Free functions rather than scheduler methods
 * because none of them is about ONE core's state, and all four are DEFINED IN
 * waitable.cpp alongside the queue machinery that drives them.
 * ------------------------------------------------------------------------- */

/**
 * @brief A thread's urgency, computed from truth: min(base, best waiter of
 *        every base_mutex it holds).
 *
 * This is the DEFINITION of priority inheritance, evaluated rather than read.
 * There is no stored effective priority and no walk maintaining one: a cache
 * whose inputs live on other cores is what every priority-inheritance bug in
 * this project's history has been.
 *
 * Callable by ANY core with no lock held. That is not incidental: held_slots is
 * a bounded array of atomics rather than an intrusive list precisely so a
 * remote core can read what a thread holds without taking its pi_lock, and
 * queue tops are maintained atomics for the same reason. A racing release
 * yields a stale-but-plausible answer, which is the tolerance the value-free
 * doorbell already rests on.
 *
 * Cost: one relaxed load and a compare when the thread holds nothing, which is
 * 98.9 percent of picks in the only test that exercises mutexes at all and 100
 * percent everywhere else. Otherwise one iteration per resource actually held,
 * measured at 1.01 resources per holder.
 */
[[nodiscard]] std::uint8_t urgency(thread_control_block const& tcb) noexcept;

/**
 * @brief urgency() with an explicit recursion budget.
 *
 * A boosted holder can itself be blocked on something it does not own, whose
 * queue contains another holder, and so on: that chain is transitive priority
 * inheritance. Following it needs a bound, because a wait-for CYCLE is a
 * deadlocked application and the kernel must not walk one forever.
 *
 * CALLER MUST NOT HOLD A QUEUE LOCK. The recursion reaches wait_queue::top,
 * which takes one to snapshot the bridges parked on that queue. It releases it
 * before recursing, so queue locks are never NESTED and a wait-for cycle is
 * merely observed rather than deadlocked on, but that only holds while every
 * caller enters from outside. In particular nothing inside a wake_one_and_commit
 * callback may ask for a thread's urgency, and thread::get_priority() is exactly
 * that question.
 *
 * held_slots is a bounded array of atomics so the fold itself needs no lock, and
 * a torn read yields a stale-but-plausible urgency, the same tolerance the rest
 * of this design rests on.
 */
[[nodiscard]] std::uint8_t urgency_at(thread_control_block const& tcb, unsigned depth) noexcept;

/**
 * @brief Commit to parking, unless a wake has already revoked the intent.
 *
 * The subtle half of the two-phase block, and the ONLY part of it that is worth
 * sharing between the group wait and a primitive with its own blocking loop. The
 * arming loops differ enough (one source versus many) that a shared abstraction
 * would cost more readability than it saves, but this test-and-commit must be
 * identical everywhere or the lost-wakeup window reopens.
 *
 * Preemption is disabled across the test and the store, so a wake cannot land
 * between deciding to block and recording it. A wake that arrived earlier has
 * already cleared the disposition, and that is what this re-reads: seeing
 * anything other than prepared means someone readied us after we armed, so we
 * must not park. The pend is deferred to preempt_enable by construction.
 *
 * Caller must be armed on every source it intends to wake from BEFORE calling.
 */
void commit_to_block(thread_control_block& tcb);

/**
 * @brief Ask @p tcb's core to reconsider what it is running.
 *
 * A pure hint carrying nothing. Used when a thread's urgency has RISEN because a
 * waiter arrived on something it holds: the new urgency is already visible in
 * that resource's queue top, but the owning core will not notice until it picks
 * again. Losing this costs latency, never correctness, because the next pick for
 * any reason folds the same truth.
 *
 * Only boosts need it. A de-boost that is observed late merely means the holder
 * ran at its old urgency slightly longer, which is safe.
 */
void request_repick(thread_control_block& tcb);

} // namespace cyros

#endif // CYROS_SCHEDULER_HPP