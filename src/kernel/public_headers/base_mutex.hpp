#ifndef CYROS_BASE_MUTEX_HPP
#define CYROS_BASE_MUTEX_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/visibility.hpp>

#include <atomic>
#include <cstdint>

namespace cyros
{


/**
 * @brief Thread-ownable resource with priority-inheritance semantics.
 *
 * `base_mutex` is the kernel's half of the mutex. The sync/ userlib feature
 * uses this interface to build the mutex types that users consume. Therefore,
 * this class is an abstract-base-class that is not intended to be instantiated
 * itself.
 *
 * Why this is separate from the generic waitable interface:
 * Mutexes are considered the special child of Cyros. They require careful handling
 * and synchronisation between threads and cores. Regular waitables don't require
 * this machinery and are not burdened with the overhead.
 * Priority transfer protocols are built-in to the `base_mutex` allowing for
 * derived mutex types to implement priority inheritance protocols (chained/transitive)
 * and priority ceiling protocols.
 * It reuses some of the same machinery as the waitable class under the hood, but
 * group composition with waiting on multiple mutexes is intentionally prohibited -
 * unlike waitables.
 *
 * Care is taken so that the complexity and overhead of acquiring and releasing mutexes
 * are only paid for when it is necessary.
 *
 * @note The `base_mutex`'s lifetime must outlive all threads interacting with it. It is recommended
 * to be placed in static storage.
 *
 * @warning Not ISR-safe: acquisition and release take the calling thread's pi_lock, which
 * assumes thread context.
 */
class CYROS_PUBLIC base_mutex
{
public:
   base_mutex(base_mutex&&) = delete;
   base_mutex(base_mutex const&) = delete;
   base_mutex& operator=(base_mutex&&) = delete;
   base_mutex& operator=(base_mutex const&) = delete;

   /**
    * @brief Block until this thread owns the resource.
    *
    * Barge-free: a release commits ownership to the chosen waiter under the
    * queue lock, so no fresh caller can interpose between the release and the
    * woken thread running.
    */
   void lock() noexcept;

   /**
    * @brief Non-blocking CAS take of a free resource.
    * @return true when ownership was acquired.
    *
    * Never arms, so unlike lock() it cannot donate and cannot park.
    */
   [[nodiscard]] bool try_lock() noexcept;

   /**
    * @brief Hand to the best waiter, or free. Owner only.
    */
   void unlock(reschedule_policy policy = reschedule_policy::automatic) noexcept;

protected:
   ~base_mutex();
   constexpr base_mutex() noexcept = default;

private:
   wait_queue queue;
   std::atomic<thread_control_block*> owner{nullptr};

   static constexpr std::uint8_t not_held = 0xFFu;

   /* Index of this resource's slot in its owner's held_slots, or not_held.
    * Written from two contexts on two cores, the owner registering under its
    * pi_lock and a releasing core committing a handover under the queue lock,
    * so it is atomic to make the cross-thread handoff formal. Relaxed is
    * enough. The two contexts are mutually excluded by the owner word's
    * transitions, only one of them can be claiming this mutex at a time, and
    * the new owner observes the value through the happens-before of the
    * scheduler wake that publishes the handover rather than through either
    * lock. Codegen for a relaxed byte is identical to a plain one. */
   std::atomic<std::uint8_t> held_slot{not_held};

   void claim_slot(thread_control_block& tcb) noexcept;

   void retire_held(thread_control_block& tcb) noexcept;

   void register_held(thread_control_block& tcb) noexcept;

   bool acquire_condition(thread_control_block& tcb) noexcept;

   void hand_over(reschedule_policy policy) noexcept;

   friend struct waitable_access;
};

} // namespace cyros

#endif // CYROS_BASE_MUTEX_HPP
