#ifndef CYROS_WAITABLE_HPP
#define CYROS_WAITABLE_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/function.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/kernel/visibility.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace cyros
{

/* ============================================================================
 * waitable - kernel base class for blockable objects
 *
 * Inherit from waitable to make your object parkable. The base class owns
 * a private wait_queue and exposes:
 *   - block()             : stateless park; wake on any signal (caller accepts
 *                           the possibility of missed events - see below)
 *   - block_until(pred)   : park until a caller-supplied predicate holds
 *   - wake_one/wake_all   : protected; for derived classes to signal waiters
 *   - is_satisfied(self)  : virtual; overridden by derived primitives so
 *                           block_on_any can poll them
 *
 * Block-on-any is provided as a free function (block_on_any) that is a
 * friend of waitable.
 *
 * Composition vs inheritance
 * --------------------------
 * Inherit waitable when your object IS something threads block ON
 * (mutex, semaphore, thread-termination, event, timer-source). Use plain
 * composition - an waitable member - when your object merely USES blocking
 * internally without being a wait target itself.
 *
 * The lost-wakeup problem and the two-phase block
 * -----------------------------------------------
 * A thread cannot simply "check, then park": between the check and the park,
 * another context may signal the condition, and the wakeup is lost. waitable
 * solves this with a two-phase block performed internally by block_until():
 *
 *   1. arm   : record intent to block, under the queue lock. Any signal
 *              AFTER this point is guaranteed to reach the caller.
 *   2. check : evaluate the predicate. If true, abandon the block.
 *   3. park  : commit to blocking. A signal that arrived since arming is
 *              honoured, not lost. Loops on spurious wakeup.
 *
 * The raw three-phase primitives are NOT exposed.
 *
 * Spurious wakeups
 * ----------------
 * A woken thread is NOT guaranteed that its condition holds: the resource may
 * have been consumed by a higher-priority waiter, or by a fresh caller racing
 * in, before the woken thread runs. block_until() and block_on_any() handle
 * this by re-checking and re-blocking transparently. This is intrinsic to all
 * blocking primitives (cf. the mandatory while-loop around pthread_cond_wait)
 * and is not a Cyros-specific cost.
 *
 * Barging
 * -------
 * waitable is BARGE-PERMITTING by design: a wake does not reserve the
 * resource for the woken thread, so a fresh thread may acquire it between
 * the wake and the woken thread running. This is cheap, high-throughput, and
 * sufficient for many primitives (counting semaphore, event, join). If your
 * primitive requires strict, barge-free, priority-fair handoff, release it
 * through wake_one_and_transfer, which commits ownership to the chosen
 * waiter under the queue lock. If it additionally requires priority
 * inheritance (typically a mutex that must bound priority inversion), derive
 * from pi_waitable below, which packages the ownership protocol and the
 * inheritance bookkeeping together.
 *
 * Concurrency & ISR safety
 * ------------------------
 * wake_one() and wake_all() are safe to call from ISR context. A wake from
 * an ISR (or any context that cannot synchronously switch) defers its
 * reschedule via the weak reschedule contract - see reschedule_policy. List
 * mutation is protected by a per-queue lock; the queue is safe under SMP
 * and against concurrent block()/wake() on different cores.
 * ========================================================================= */
class CYROS_PUBLIC waitable
{
public:
   /**
    * @brief Caller's instruction for whether signalling should trigger a
    *        reschedule on the *local* core.
    */
   enum class reschedule_policy
   {
      automatic, ///< Conditionally trigger a local reschedule if a better thread is made ready (preferred)
      never,     ///< Never trigger a local reschedule
      always,    ///< Always trigger a local reschedule
   };

   virtual ~waitable();

   waitable(waitable&&) = delete;
   waitable(waitable const&) = delete;
   waitable& operator=(waitable&&) = delete;
   waitable& operator=(waitable const&) = delete;

protected:
   using transfer_fn = function<void(uint32_t), 32, heap_policy::no_heap>;

   waitable() noexcept = default;

   /**
    * @brief Override in derived primitives so wait_on_any can poll them.
    *
    * @param caller The thread evaluating the predicate (always the currently
    *               running thread on this core).
    *
    * @return true if waiter will not block (e.g. because it acquired the resource)
    *         false if it will block.
    */
   virtual bool wait_condition(thread& caller) noexcept = 0;

   /**
    * @brief Wake the single highest-priority waiting thread (if any).
    *
    * @param policy After waking a waiting thread, apply this reschedule policy.
    *
    * No-op if no waiters. Safe within ISR context.
    * Woken thread may be barged by another thread. This reduces latency
    * at the cost of waiter fairness.
    */
   void wake_one(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Wake ALL waiting threads (if any).
    *
    * @param policy After waking a waiting thread, apply this reschedule policy.
    *
    * No-op if no waiters. Safe from ISR context.
    * Threads are woken as a batch with preemption disabled
    * before releasing all at once.
    */
   void wake_all(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Wake-and-handover to the single highest-priority waiting thread (if any).
    *
    * @param transfer Callable that hands over resource ownership to woken thread.
    * @param policy After waking a waiting thread, apply this reschedule policy.
    * @return true when a waiter was chosen and readied, false when the
    *         queue was empty and transfer received 0. Discardable.
    *
    * No-op if no waiters. Safe from ISR context.
    * The barge-free sibling of wake_one(). Pops the best waiter and invokes
    * transfer exactly once under the queue lock, passing the waiter's thread
    * id, or 0 when no thread is parked. transfer must record the resource
    * state for BOTH outcomes, e.g. owner = next_owner_id. Committing the
    * empty case under the same lock is what closes the lost wakeup where a
    * waiter arms, polls the still-held resource, and parks just before this
    * release frees it.
    *
    * A woken thread finds the resource already assigned to it, so the
    * derived is_satisfied() must recognise ownership by id in addition to
    * taking the resource when free. No fresh caller can interpose between
    * the release and the woken thread running, which is the barge-free
    * guarantee.
    *
    * transfer runs under a spinlock. It must be tiny, must not block, must
    * not wake, and must not touch this waitable's queue.
    */
   bool wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy = reschedule_policy::automatic) noexcept;

   /**
    * @brief Priority-inheritance chain hook.
    *
    * Invoked by the priority recompute when this waitable's best-waiter
    * priority changed while a thread was re-slotted in its queue. Return the
    * thread that inherits from this waitable's waiters (its current holder)
    * so the walk can chase the chain, or nullptr for ownerless waitables,
    * which is the default because a semaphore, event, or join has nobody to
    * inherit anything.
    *
    * The id written through expected_id validates the returned TCB against
    * recycling before it is acted on. Called without any pi_lock held.
    */
   virtual thread_control_block* donation_target(thread::id& expected_id) noexcept
   {
      (void)expected_id;
      return nullptr;
   }

   /**
    * @brief Group-wait handback hook.
    *
    * A transfer can commit ownership to a thread parked in wait_on_any at any
    * moment while its node is armed, so a group wait can wake owning MORE
    * than the waitable it returns. The contract is that it owns exactly the
    * returned index (plus anything it held before the call), so wait_on_any
    * sweeps the non-chosen waitables through this hook after the final
    * disarm, when no further transfer can choose the caller. Ownerless
    * waitables have nothing to hand back, hence the default no-op.
    */
   virtual void renounce_if_assigned(thread::id thread_id) noexcept
   {
      (void)thread_id;
   }

private:
   /**
    * @brief Per-thread parking record (one per TCB, reused).
    *
    * Lives in the TCB, threaded into one or more waitable::wait_queue
    * instances when the thread is parked.
    */
   struct wait_node
   {
      thread_control_block* owner{nullptr};
      wait_node*            next {nullptr};
      waitable*             source{nullptr};

      // For wait_on_any: which waitable does this slot in the call's source
      // array correspond to. Unused (and zero) in single-wait blocks.
      uint8_t source_index{0};
   };
   friend class wait_node_vector;

   /**
    * @brief Private intrusive priority-ordered list of waiting threads.
    */
   class CYROS_PUBLIC wait_queue
   {
   public:
      using commit_fn = function<void(thread_control_block*), 32, heap_policy::no_heap>;
      static constexpr std::uint8_t no_waiter = 0xFF;

      void arm   (wait_node& n) noexcept;
      bool disarm(wait_node& n) noexcept;
      bool reslot(wait_node& n) noexcept;

      void wake_one(reschedule_policy) noexcept;
      void wake_all(reschedule_policy) noexcept;
      bool wake_one_and_transfer(transfer_fn const&, reschedule_policy) noexcept;
      bool wake_one_and_commit(commit_fn const& commit, reschedule_policy policy) noexcept;

      [[nodiscard]] bool empty() const noexcept;
      [[nodiscard]] std::uint8_t top() const noexcept { return top_priority.load(std::memory_order_acquire); }


   private:
      void link  (wait_node&) noexcept;
      bool unlink(wait_node&) noexcept;
      void refresh_top()      noexcept;

      spinlock   lock;
      wait_node* head{nullptr}; // priority-ordered, best at head
      std::atomic<std::uint8_t> top_priority{no_waiter};
   };
   wait_queue queue;

   friend class pi_waitable;
   friend class waitable_arm_guard;
   friend struct waitable_access;
   friend std::size_t this_thread::wait_on_any(std::span<waitable_ref>) noexcept;
};


/**
 * @brief waitable type that NEVER blocks the thread
 *
 * This allows wait_for_any({target, non_blocking_token});
 * patterns.
 */
class CYROS_PUBLIC non_blocking_token : public waitable
{
protected:
   bool wait_condition(thread&) noexcept override
   {
      return true;
   }
};


/* ============================================================================
 * pi_waitable - ownable waitable with priority inheritance
 *
 * The base for primitives that have a single owning thread at a time and must
 * bound priority inversion (typically a mutex). It packages three things:
 *
 *   1. The ownership protocol proven by wake_one_and_transfer: a CAS take on
 *      owner_id when free, recognition of ownership already transferred while
 *      parked, and a barge-free handover committed under the queue lock.
 *
 *   2. Donation: a thread about to park behind a live owner rings a priority
 *      recompute for that owner, so the owner runs at the urgency of its most
 *      urgent waiter.
 *
 *   3. Restore: release retires the resource from the owner's held list and
 *      recomputes the owner from remaining truth, so donations never outlive
 *      the contention that justified them.
 *
 * Lock ordering
 * ----------------------------
 * The recompute nests pi_lock -> queue lock, therefore nothing running under
 * a queue lock may take any pi_lock. This is why the handover commit only
 * writes this object's own atomics and the woken thread completes its own
 * held-list linkage in its next wait_condition poll, in its own context.
 *
 * Not ISR-safe: acquisition and release take the calling thread's pi_lock and
 * recompute, both of which assume thread context.
 *
 * A thread must not terminate while owning a pi_waitable ,
 * and a pi_waitable must not be destroyed while owned.
 * ========================================================================= */
class CYROS_PUBLIC pi_waitable : public waitable
{
public:
   ~pi_waitable() override;

protected:
   pi_waitable() noexcept = default;

   /**
    * @brief Non-blocking CAS take of a free resource by the calling thread.
    * @return true when ownership was acquired.
    */
   [[nodiscard]] bool pi_try_acquire() noexcept;

   /**
    * @brief The wait_condition body for an owned resource.
    *
    * Takes the resource when free, recognises ownership already transferred
    * to the caller while it was parked, and otherwise donates to the live
    * owner before the caller parks.
    *
    * @return true if the caller owns the resource and will not block.
    */
   bool pi_acquire_condition(thread& caller) noexcept;

   /**
    * @brief Release: barge-free handover to the best waiter, or free.
    *
    * Retires the resource from the caller's held list, commits the handover
    * (or the free) under the queue lock, then restores the caller's priority
    * from remaining truth.
    */
   void pi_release(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   thread_control_block* donation_target(thread::id& expected_id) noexcept override;

   void renounce_if_assigned(thread::id thread_id) noexcept override;

private:
   std::atomic<thread::id> owner_id{0}; // 0 when free, otherwise owner's thread id
   std::atomic<thread_control_block*> holder{nullptr}; // The owner's TCB for donation targeting
   /* Index of this resource's slot in its owner's held_slots, or not_held.
    * Replaces the old intrusive next_held link and its self-sentinel, keeping
    * registration, retirement and the "am I registered" test all O(1). */
   static constexpr std::uint8_t not_held = 0xFFu;
   std::uint8_t held_slot{not_held};

   void register_held(thread_control_block& tcb) noexcept;

   // The shared release commit: hand to the best waiter or free, under the
   // queue lock. Used by pi_release (registered ownership) and
   // renounce_if_assigned (in-flight assignment a group wait declined).
   void hand_over(reschedule_policy policy) noexcept;

   friend struct waitable_access;
};




} // namespace cyros

#endif // CYROS_WAITABLE_HPP
