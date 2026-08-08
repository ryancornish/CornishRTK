#ifndef CYROS_SCHEDULER_HPP
#define CYROS_SCHEDULER_HPP

#include "threading_subsystem.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

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

/**
 * @brief What a core can ask another core to do to one of its threads.
 *
 * There is no message object: the TCB IS the message, and this only names which
 * bit of thread_control_block::pending_requests the request occupies. Both types
 * are value-free doorbells against a single TCB, so two of the same kind for the
 * same target are one obligation and fold into one bit. A type that ever carried
 * a value could not use this, and would need somewhere on the TCB to put it.
 */
struct cross_core_request
{
   enum class request_type : uint8_t
   {
      set_thread_ready,   // Enqueue a TCB into this core's ready queue
      recompute_priority, // Re-derive a TCB's effective priority from current truth
   };
   static constexpr auto set_thread_ready   = request_type::set_thread_ready;
   static constexpr auto recompute_priority = request_type::recompute_priority;

   [[nodiscard]] static constexpr std::uint8_t claim_bit_for(request_type t) noexcept
   {
      return static_cast<std::uint8_t>(1u << static_cast<std::uint8_t>(t));
   }
};

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

   [[nodiscard]] constexpr uint8_t current_thread_priority() const noexcept
   {
      return current_thread ? current_thread->priority() : 0;
   }

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
    * @brief Move a thread on this core to a new effective priority.
    *
    * The one place the effective_priority field and the thread's scheduling
    * position change together. The matrix keys removal on the CURRENT field
    * value, so an already-enqueued thread is removed at the old value,
    * rewritten, and re-enqueued at the new. Must run on the owning core, the
    * caller holds the thread's pi_lock, whose interrupt-masking grade makes
    * the matrix surgery atomic against ISRs as well as thread switches.
    *
    * @return warranted when the change makes a reschedule worthwhile: a ready
    *         thread was raised above the running one, or the running one
    *         dropped below a ready peer.
    */
   schedule_hint reprioritise_thread(thread_control_block& tcb, uint8_t new_effective) noexcept;

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
   void post_intake(thread_control_block& tcb, cross_core_request::request_type type) noexcept;

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