#ifndef CYROS_THREADING_SUBSYSTEM_HPP
#define CYROS_THREADING_SUBSYSTEM_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>

#include "relaxed_atomic.hpp"

#include <atomic>
#include <array>
#include <bitset>
#include <limits>

namespace cyros
{

class base_mutex;

void thread_launcher(void* tcb_ptr);

class thread_termination final : public waitable
{
   std::atomic<bool> terminated{false};

protected:
   bool try_satisfy() noexcept override
   {
      return terminated.load(std::memory_order_acquire);
   }

public:
   void terminate()
   {
      bool expected = false;
      CYROS_ASSERT(terminated.compare_exchange_strong(expected, true, std::memory_order_release));

      wake_all(reschedule_policy::never);
   }
};

enum class thread_state : uint8_t
{
   created,
   ready,
   running,
   blocked,
   terminated,
};

/**
 * @brief Work one core can ask the owning core to do to a thread.
 *
 * Not a message type: there is no message. Each enumerator names one bit of
 * thread_control_block::pending_requests, which IS the request. Both are
 * value-free, so two of the same kind against one thread are one obligation and
 * collapse into one bit. Anything that needed to carry a value would need a home
 * on the TCB, not an entry in a queue.
 */
enum class thread_request : std::uint8_t
{
   make_ready, ///< Admit this thread to its core's runnable set
};

enum class thread_disposition : uint8_t
{
   none,        ///< No pending wish - scheduled purely on position.
   prepared,    ///< Wishes to block but still deciding - stays runnable if preempted.
   committed,   ///< Decision made under preempt-disable - reschedule will park it.
   terminating, ///< Finished. The arbiter retires it. Outranks state, never revoked.
};

/**
 * @brief Maximum base_mutexes one thread may own at once.
 *
 * Deliberately a kernel constant and NOT user configuration. The bound has to
 * exist and be stated (it is what makes the urgency fold's worst case a number
 * rather than "usually small"), but nothing yet suggests users need to tune it,
 * and every knob is a cost. Promote it to config::max_held_per_thread only when
 * there is evidence a real system needs a different value.
 *
 * Exceeding it is a hard error rather than a silent drop: a lost held resource
 * means a lost donation, which is exactly the failure class this design exists
 * to remove.
 */
inline constexpr std::size_t max_held_per_thread = 4;

struct thread_control_block
{
   relaxed_atomic<thread_state>       state{thread_state::created};
   relaxed_atomic<thread_disposition> disposition{thread_disposition::none};

   /* Intrusive link for a thread_ready_queue. Self-pointer is the not-enqueued
    * sentinel. Singly linked: the queue needs only push_back and pop_front now
    * that nothing re-keys a thread's position, so a back pointer would cost 8
    * bytes to serve no caller. */
   thread_control_block* next{this};


   thread::id id{0};

   /* The thread's own priority, fixed for its lifetime.
    *
    * There is no effective_priority field. Urgency is min(base, best waiter of
    * every base_mutex held) and is COMPUTED at the point of use by
    * thread_action::urgency(), never stored. A stored copy would be a cache whose
    * inputs live on other cores, which is what every priority-inheritance bug in
    * this project's history has been. */
   uint8_t base_priority;
   thread* public_thread_handle;

   // Core pinning
   std::uint32_t pinned_core{0};
   core_affinity  affinity;

   /* Priority inheritance ---------------------------------------------------
    * pi_lock serialises this thread's OWN mutation of held_slots and held_mask
    * against itself. Nothing holding a wait_queue lock may take it, which is why
    * a handover files its slot by CAS instead. Reads need no lock at all, which
    * is the whole point: see the ordering note on held_mask. */
   spinlock pi_lock;

   /* The resources this thread currently owns.
    *
    * A bounded array of independently readable slots rather than an intrusive
    * list, for one reason: a list can only be traversed by a core holding
    * pi_lock, so a remote core cannot read what this thread holds without
    * taking a remote lock. A fixed array of atomics can be scanned by anyone,
    * which is what lets a thread's urgency be computed at the point of use
    * instead of cached and propagated. See pi-design-principles.md.
    *
    * Two contexts write a slot: this thread, and a core committing a handover
    * to it under a queue lock, which cannot reach pi_lock. Both claim by CAS so
    * neither needs a lock the other cannot take. Reads are always unlocked, and
    * a reader may observe a slot that has just been released. That degrades to
    * a stale-but-plausible urgency, which is the same tolerance the value-free
    * doorbell already relies on.
    *
    * A free slot is nullptr. base_mutex::held_slot indexes back into here so
    * registration and retirement stay O(1), as the list's self-sentinel did. */
   std::array<std::atomic<base_mutex*>, max_held_per_thread> held_slots{};

   /* Occupancy bitmask over held_slots, so the overwhelmingly common "this
    * thread holds nothing" case is one load and one compare instead of a walk
    * over every slot, and an occupied scan costs one iteration per resource
    * actually held rather than max_held_per_thread.
    *
    * Measured before this existed: scanning eight empty slots cost 91 cycles
    * against 106 for finding one resource, so the scan WAS the cost, not the
    * work. Threads that hold anything hold 1.01 resources on average.
    *
    * ORDERING, which is what makes it safe to read without pi_lock:
    *   register: publish the slot, THEN set the bit
    *   release:  clear the bit, THEN clear the slot
    * A reader that sees a stale set bit loads nullptr and skips. A reader that
    * misses a just-set bit computes a slightly stale urgency, which is the
    * tolerance this whole design already rests on.
    *
    * A slot is claimed by CAS rather than under pi_lock alone, because a
    * releasing core files a handover into the new owner's slots from under a
    * queue lock, where that owner's pi_lock is out of reach. */
   std::atomic<std::uint8_t> held_mask{0};
   static_assert(max_held_per_thread <= 8, "held_mask is a uint8_t, so at most 8 slots");

   /* The mutex this thread is currently parked on, or nullptr.
    *
    * The one edge that makes the wait-for graph walkable FROM a thread. Without
    * it the only direction available is queue to owner, so a prompt aimed at a
    * blocked holder has nowhere to go and the far end of a chain is unreachable,
    * which is why request_repick had to broadcast to every core.
    *
    * Published AFTER the node is armed and cleared BEFORE the queue is left, so
    * it is set exactly while this thread is queued. Atomic because the walk
    * reads it from other cores. A stale read costs a redundant or a missed
    * prompt, never a wrong value. */
   std::atomic<base_mutex*> blocked_on{nullptr};

   /* Cross-core requests outstanding against this thread, one bit per type.
    *
    * This is the request itself, not a claim on a slot elsewhere: the TCB is the
    * message. A producer that finds the whole field zero knows this TCB is not
    * on any core's intake and must put it there. Any non-zero value means it
    * already is, and since these requests carry truth rather than values, the
    * entry already queued discharges the new intent too.
    *
    * ORDERING:
    *   producer: set the bit, THEN push onto the intake
    *   consumer: take the bits, THEN service
    * A set bit therefore guarantees a service strictly in the observer's future.
    * Servicing before taking would fold a request raised mid-service into work
    * already done, which is a lost wake.
    */
   std::atomic<std::uint8_t> pending_requests{0};

   [[nodiscard]] static constexpr std::uint8_t request_bit(thread_request r) noexcept
   {
      return static_cast<std::uint8_t>(1u << static_cast<std::uint8_t>(r));
   }

   /* Link in the owning core's intake stack. The TCB IS the cross-core message,
    * so there is no queue to overflow and no capacity to tune.
    *
    * A producer publishes the node's POSITION (the exchange) before its LINK
    * (this store), so between the two the node is reachable and points nowhere.
    * SELF-POINTER MEANS "not linked yet, wait". nullptr cannot serve as that
    * marker, because the bottom of every chain legitimately ends in nullptr, and
    * a consumer that read nullptr there would stop early and strand every node
    * beneath it.
    *
    * The consumer's wait is bounded ONLY because the push runs with interrupts
    * masked, so the producer cannot be preempted between the two stores. Remove
    * that and the wait becomes unbounded rather than merely slower, which is the
    * whole justification for using an exchange instead of a CAS loop. See
    * scheduler::push_intake. */
   std::atomic<thread_control_block*> intake_next{nullptr};

   /* Link in the owning core's list of READY threads that hold at least one
    * base_mutex. Self-pointer is the not-linked sentinel, as elsewhere.
    *
    * Membership is a property of being ready, not of holding: a thread joins at
    * set_thread_ready and leaves when picked. It is never listed while running,
    * which is why acquiring and releasing do not touch this at all.
    *
    * These are exactly the threads whose urgency can differ from their base
    * priority, so once the ready matrix is keyed on base_priority this is the
    * only set the pick has to fold urgency over. Holders are rare, measured at
    * 1.09 percent of picks in the one test that exercises mutexes at all and
    * zero everywhere else, so the list is almost always empty and the pick
    * degenerates to the matrix head.
    *
    * Needs no atomics: membership changes only in set_thread_ready and
    * pick_next, both of which run on the thread's PINNED core, and the only
    * reader is that same core's pick. */
   thread_control_block* holder_next{this};

   [[nodiscard]] constexpr bool is_listed_holder() const noexcept
   {
      return holder_next != this;
   }

   std::span<std::byte> stack;
   thread::entry_fn entry;

   // Thread-joining waitable
   thread_termination termination;

   // Opaque, in-place port context storage
   alignas(CYROS_PORT_CONTEXT_ALIGN) std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> context_storage{};
   [[nodiscard]] constexpr auto*       context()       noexcept { return reinterpret_cast<cyros_port_context_t*      >(context_storage.data()); }
   [[nodiscard]] constexpr auto const* context() const noexcept { return reinterpret_cast<cyros_port_context_t const*>(context_storage.data()); }

   [[nodiscard]] constexpr bool is_enqueued() const noexcept
   {
      return next != this;
   }

   /**
    * @brief True when this thread owns no base_mutex. One load, one compare.
    */
   [[nodiscard]] constexpr bool holds_nothing() const noexcept
   {
      return held_mask.load(std::memory_order_relaxed) == 0;
   }

   /**
    * @brief Claim the right to queue @p bit. True when the caller must queue.
    */
   [[nodiscard]] bool claim_request(std::uint8_t bit) noexcept
   {
      return (pending_requests.fetch_or(bit, std::memory_order_acq_rel) & bit) == 0;
   }

   /**
    * @brief Release a claim. Call BEFORE servicing, or before abandoning a claim.
    */
   void release_request(std::uint8_t bit) noexcept
   {
      pending_requests.fetch_and(static_cast<std::uint8_t>(~bit), std::memory_order_release);
   }

   thread_control_block(thread::priority priority,
                        core_affinity affinity,
                        std::span<std::byte> stack,
                        thread::entry_fn&& entry,
                        thread* public_thread_handle);
};


/**
 * Carves a user-provided buffer region into:
 * +----------------------+ <-- buffer's end (high address)
 * +   thread_control_block   + (Fixed size)
 * +----------------------+
 * + Thread-local storage + (Variable size)
 * +----------------------+
 * +     User's stack     +
 * +----------------------+ <-- buffer's base (low address)
 */
struct stack_layout
{
   thread_control_block* tcb;
   std::span<std::byte> tls_region;
   std::span<std::byte> user_stack;

   explicit stack_layout(std::span<std::byte> buffer, std::size_t tls_bytes);
};

class thread_ready_queue
{
private:
   thread_control_block* head{nullptr};
   thread_control_block* tail{nullptr};

public:
   [[nodiscard]] constexpr bool empty() const noexcept
   {
      return !head;
   }

   void push_back(thread_control_block& tcb) noexcept;

   thread_control_block* pop_front() noexcept;

   bool remove(thread_control_block& tcb) noexcept;
};

class thread_ready_matrix
{
private:
   std::array<thread_ready_queue, config::max_priorities> matrix{};
   std::uint32_t bitmap{0};
   static_assert(config::max_priorities <= std::numeric_limits<decltype(bitmap)>::digits,
                 "bitmap cannot represent that many priorities!");

public:
   constexpr thread_ready_matrix() = default;

   [[nodiscard]] constexpr int best_priority() const noexcept
   {
      return bitmap ? std::countr_zero(bitmap) : -1;
   }

   [[nodiscard]] constexpr bool empty() const noexcept
   {
      return bitmap == 0;
   }

   /** @brief Enqueue at the thread's BASE priority, which never changes. */
   void enqueue_thread(thread_control_block& tcb) noexcept;

   thread_control_block* pop_best_thread() noexcept;

   /** @brief Unfile an enqueued thread, keyed at its BASE priority. */
   void remove_thread(thread_control_block& tcb) noexcept;

};

} // namespace cyros

#endif // CYROS_THREADING_SUBSYSTEM_HPP
