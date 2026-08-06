#ifndef CYROS_THREADING_SUBSYSTEM_HPP
#define CYROS_THREADING_SUBSYSTEM_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>

#include <atomic>
#include <array>
#include <bitset>
#include <limits>

namespace cyros
{

class wait_node_vector;

void thread_launcher(void* tcb_ptr);

class thread_termination final : public waitable
{
   std::atomic<bool> terminated{false};

protected:
   bool wait_condition(thread& caller) noexcept override
   {
      (void)caller;
      return terminated.load(std::memory_order_acquire);
   }

public:
   void terminate()
   {
      bool expected = false;
      CYROS_ASSERT(terminated.compare_exchange_strong(expected, true, std::memory_order_release));

      // terminate() is invoked on the teardown path of the thread launcher.
      // Invoking a reschedule is forbidden during this period as it as we might
      // otherwise switch away and never return to continue the teardown.
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

enum class thread_disposition : uint8_t
{
   none,      ///< No pending wish - scheduled purely on position.
   prepared,  ///< Wishes to block but still deciding - stays runnable if preempted.
   committed, ///< Decision made under preempt-disable - reschedule will park it.
};

/**
 * @brief Maximum pi_waitables one thread may own at once.
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

   thread_state state{thread_state::created};
   thread_disposition disposition{thread_disposition::none};

   // Intrusive 'linked-list' links for a thread_ready_queue. Pointing to self
   // represents the not-enqueued sentinel
   thread_control_block* next{this};
   thread_control_block* prev{this};


   thread::id id{0};
   uint8_t base_priority;
   std::atomic<uint8_t> effective_priority; // Can change dynamically
   thread* public_thread_handle;

   // Core pinning
   std::uint32_t pinned_core{0};
   core_affinity  affinity;

   /* Priority inheritance ---------------------------------------------------
    * pi_lock protects the held list and the active_waits registration. It is
    * the OUTER lock of the pi_lock -> queue-lock ordering, nothing holding a
    * wait_queue lock may take it. */
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
    * MUTATION still happens only under pi_lock, in the owner's own context.
    * Only reads are unlocked, and a reader may observe a slot that has just
    * been released. That degrades to a stale-but-plausible urgency, which is
    * the same tolerance the value-free doorbell already relies on.
    *
    * A free slot is nullptr. pi_waitable::held_slot indexes back into here so
    * registration and retirement stay O(1), as the list's self-sentinel did. */
   std::array<std::atomic<pi_waitable*>, max_held_per_thread> held_slots{};

   /* Occupancy bitmask over held_slots, so the overwhelmingly common "this
    * thread holds nothing" case is one load and one compare instead of a walk
    * over every slot, and an occupied scan costs one iteration per resource
    * actually held rather than max_held_per_thread.
    *
    * Measured before this existed: scanning eight empty slots cost 91 cycles
    * against 106 for finding one resource, so the walk WAS the cost, not the
    * work. Threads that hold anything hold 1.01 resources on average.
    *
    * ORDERING, which is what makes it safe to read without pi_lock:
    *   register: publish the slot, THEN set the bit
    *   release:  clear the bit, THEN clear the slot
    * A reader that sees a stale set bit loads nullptr and skips. A reader that
    * misses a just-set bit computes a slightly stale urgency, which is the
    * tolerance this whole design already rests on. */
   std::atomic<std::uint8_t> held_mask{0};
   static_assert(max_held_per_thread <= 8, "held_mask is a uint8_t, so at most 8 slots");

   /** @brief True when this thread owns no pi_waitable. One load, one compare. */
   [[nodiscard]] bool holds_nothing() const noexcept
   {
      return held_mask.load(std::memory_order_relaxed) == 0;
   }

   wait_node_vector* active_waits{nullptr};


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

   [[nodiscard]] constexpr uint8_t priority() const
   {
      return effective_priority.load(std::memory_order_relaxed);
   }

   [[nodiscard]] bool is_higher_priority_than(thread_control_block& rhs) const noexcept
   {
      return priority() < rhs.priority();
   }

   [[nodiscard]] constexpr bool is_higher_priority_than(uint8_t priority_level) const noexcept
   {
      return priority() < priority_level;
   }

   constexpr void set_priority(uint8_t p)
   {
      effective_priority.store(p, std::memory_order_relaxed);
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

   [[nodiscard]] constexpr bool has_peer() const noexcept
   {
      return head && head != tail;
   }

   [[nodiscard]] constexpr thread_control_block* front() const noexcept
   {
      return head;
   }

   // This walks the linked list so isn't 'free'
   [[nodiscard]] constexpr std::size_t size() const noexcept
   {
      std::size_t n = 0;
      for (auto* tcb = head; tcb; tcb = tcb->next) ++n;
      return n;
   }

   void push_back(thread_control_block& tcb) noexcept;

   thread_control_block* pop_front() noexcept;

   void remove(thread_control_block& tcb) noexcept;
};

class thread_ready_matrix
{
private:
   static constexpr std::size_t bitmap_bits = std::numeric_limits<uint32_t>::digits;
   std::array<thread_ready_queue, config::max_priorities> matrix{};
   uint32_t bitmap{0};
   static_assert(config::max_priorities <= bitmap_bits, "bitmap cannot hold that many priorities!");

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

   [[nodiscard]] constexpr bool empty_at(uint32_t priority) const noexcept
   {
      return matrix[priority].empty();
   }

   [[nodiscard]] constexpr bool has_peer(uint32_t priority) const noexcept
   {
      return matrix[priority].has_peer();
   }

   [[nodiscard]] constexpr std::size_t size_at(uint32_t priority) const noexcept
   {
      return matrix[priority].size();
   }

   [[nodiscard]] constexpr std::bitset<bitmap_bits> bitmap_view() const noexcept
   {
      return {bitmap};
   }

   [[nodiscard]] constexpr thread_control_block* peek_best_thread() const noexcept
   {
      if (bitmap == 0) return nullptr;
      auto const priority = std::countr_zero(bitmap);
      return matrix[priority].front();
   }

   void enqueue_thread(thread_control_block& tcb) noexcept;

   thread_control_block* pop_best_thread() noexcept;

   void remove_thread(thread_control_block& tcb) noexcept;
};

} // namespace cyros

#endif // CYROS_THREADING_SUBSYSTEM_HPP
