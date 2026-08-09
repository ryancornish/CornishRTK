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
 * @brief TODO
 */
enum class [[nodiscard]] schedule_hint
{
   unwarranted, ///< TODO
   warranted,   ///< TODO
};

void idle_task();

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
    * to look at. So there is no capacity, nothing to overflow, and no correct-
    * response-to-full problem, which is what the bounded ring had and could not
    * answer.
    *
    * A port whose CPU has no lock-free RMW (Cortex-M0/M0+ has no LDREX/STREX)
    * supplies its own __atomic_* implementations, which fixes every atomic in
    * the kernel at once rather than this one. See port.h. */
   std::atomic<thread_control_block*> intake_head{nullptr};

   /* Threads pinned here that hold at least one pi_waitable, i.e. exactly the
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

public:
   static constexpr thread::id idle_thread_id = 0; // Reserved

   constexpr explicit scheduler(std::size_t core_id) : core_id(core_id) {};

   ~scheduler() = default;
   scheduler(scheduler&&) = delete;
   scheduler(scheduler const&) = delete;
   scheduler& operator=(scheduler&&) = delete;
   scheduler& operator=(scheduler const&) = delete;

   [[nodiscard]] constexpr thread::id current_thread_id() const noexcept
   {
      return current_thread ? current_thread->id : 0;
   }

   /** @brief Urgency of the running thread, folded from truth. */
   [[nodiscard]] uint8_t current_thread_urgency() const noexcept;

   /** @brief Threads pinned here. Used by pin_thread_to_core to load balance. */
   [[nodiscard]] uint32_t pinned_thread_count() const noexcept
   {
      return pinned_thread_counter.load(std::memory_order_relaxed);
   }

   [[nodiscard]] constexpr thread_control_block& get_current_thread() const noexcept
   {
      CYROS_ASSERT(current_thread != nullptr); // Not invocable from non-thread context

      return *current_thread;
   }

   void pin_thread(thread_control_block& tcb);

   void init_idle_thread();

   // Core-local operations (only called on owning core)
   void start() noexcept;

   schedule_hint set_thread_ready(thread_control_block& tcb) noexcept;

   void set_thread_running(thread_control_block& tcb) noexcept;

   void set_thread_blocked(thread_control_block& tcb) noexcept;

   void set_thread_terminated(thread_control_block& tcb) noexcept;

   /**
    * @brief Choose the most urgent runnable thread on this core.
    *
    * Best of the matrix head (most urgent non-holder, by base priority) and the
    * most urgent holder, whose urgency is folded from truth rather than read
    * from a cache. Returns nullptr when nothing is runnable.
    */
   thread_control_block* pick_next() noexcept;

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
   [[nodiscard]] bool push_intake(thread_control_block& tcb) noexcept;

   /**
    * @brief Ask this core to service @p type for @p tcb. Any core may call.
    *
    * Claims, enqueues and notifies. There is no failure mode: the TCB is the
    * message and it always has room for itself, so nothing can overflow and
    * there is no full-queue case needing a correct response.
    */
   void post_intake(thread_control_block& tcb, thread_request request) noexcept;

   /**
    * @brief Act on every request bit set for @p tcb. Owning core only.
    */
   void service_intake(thread_control_block& tcb, std::uint8_t bits) noexcept;

   /**
    * @brief Take the whole intake chain and service every request on it.
    *
    * Single consumer: only the owning core may call this.
    */
   void drain_intake() noexcept;

   /**
    * @brief Append a ready thread that holds something to the holder list.
    *
    * Called from set_thread_ready, not from acquisition: a thread that takes a
    * resource is running, so it joins the list at its next re-ready. Idempotent.
    */
   void link_holder(thread_control_block& tcb) noexcept;

   /**
    * @brief Remove a holder from the list because the pick chose it.
    *
    * The counterpart to link_holder, and the only way out of the list. A thread
    * that releases its last resource is running, so it is already off it.
    */
   void unlink_holder(thread_control_block& tcb) noexcept;

   /**
    * @brief True when this core has intake work outstanding. Advisory.
    *
    * Cheap enough for the idle loop to check before sleeping, which is what
    * keeps a lost IPI to a scheduling round instead of a hang.
    */
   [[nodiscard]] bool intake_pending() const noexcept
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
   void reschedule() noexcept;

   void reset();
};

} // namespace cyros

#endif // CYROS_SCHEDULER_HPP