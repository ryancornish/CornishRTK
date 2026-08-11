#ifndef CYROS_BASE_MUTEX_HPP
#define CYROS_BASE_MUTEX_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/visibility.hpp>

#include <atomic>
#include <cstdint>

namespace cyros
{

/* ============================================================================
 * base_mutex - ownable resource with priority inheritance, NOT a waitable
 *
 * The kernel half of the mutex. Users do not include this header, they include
 * <cyros/sync/mutex.hpp> and get a zero-cost facade over it, the same way
 * nobody includes <cyros/kernel/waitable.hpp> to use a semaphore. The type lives
 * here because the SCHEDULER has to name it: thread_control_block::held_slots
 * holds these, and the urgency fold walks them at every pick. That is a
 * statement about who owns the data, not about who owns the surface.
 *
 * Why it is not a waitable
 * ------------------------
 * A waitable can enter a wait_on_any group. A mutex must not, and the ban is
 * structural rather than a runtime check, because the composition is not merely
 * unwanted but actively broken: a group waiter woken by its OTHER source is
 * admitted to the ready matrix holding nothing, and a handover landing on it
 * afterwards files a resource into a thread the fold no longer looks at. See
 * cross-core-defects.md 8.1. Not deriving from waitable makes that
 * unrepresentable at zero cost.
 *
 * The consequence worth knowing: there is no renounce protocol here and no
 * "assigned but unclaimed" state, because a mutex can only ever be handed to a
 * thread that is waiting for exactly it. Both existed solely to disambiguate
 * the group-wait case.
 *
 * Inheritance, and what a held resource contributes
 * -------------------------------------------------
 * A thread's urgency is min(base, contribution of every resource it holds), and
 * this type's contribution is its queue's best waiter. That is the whole of
 * priority inheritance here: nothing is stored, propagated or restored, because
 * the fold reads current truth at the point of use.
 *
 * The constructor is protected so this type cannot be instantiated bare. Today
 * there is one inversion policy and the facade in sync/ names it. When ceiling
 * lands as a sibling facade it becomes a constructor parameter plus one branch
 * in the fold, and the protected constructor is what guarantees every mutex in
 * the system states which protocol it runs. See mutex-first-class-plan.md D6.
 *
 * Not ISR-safe: acquisition and release take the calling thread's pi_lock, which
 * assumes thread context.
 * ========================================================================= */
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
   base_mutex() noexcept = default;

private:
   /// Composed, not inherited. See wait_queue for why the name is visible here
   /// and still unusable.
   wait_queue queue;

   /* One word, not two. The CAS take, the transferred-recognition test and the
    * donation target all read it, so there is no pair to keep ordered. */
   std::atomic<thread_control_block*> owner{nullptr}; // nullptr when free

   /* Index of this resource's slot in its owner's held_slots, or not_held. */
   static constexpr std::uint8_t not_held = 0xFFu;
   std::uint8_t held_slot{not_held};

   /// File this resource in @p tcb's held slots. Lock-free, see the .cpp.
   void claim_slot(thread_control_block& tcb) noexcept;

   /// Remove it again. Caller holds tcb.pi_lock.
   void retire_held(thread_control_block& tcb) noexcept;

   void register_held(thread_control_block& tcb) noexcept;

   /// Take if free, recognise ownership already transferred to us, or donate.
   bool acquire_condition(thread_control_block& tcb) noexcept;

   /// The release commit: hand to the best waiter or free, under the queue lock.
   void hand_over(reschedule_policy policy) noexcept;

   friend struct waitable_access;
};

} // namespace cyros

#endif // CYROS_BASE_MUTEX_HPP
