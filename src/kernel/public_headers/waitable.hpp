#ifndef CYROS_WAITABLE_HPP
#define CYROS_WAITABLE_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/function.hpp>
#include <cyros/kernel/spinlock.hpp>
#include <cyros/kernel/visibility.hpp>

#include <cstddef>
#include <array>
#include <cstdint>
#include <span>

namespace cyros
{

class waitable;
class pi_waitable;
class waitable_arm_guard;
class wait_node_vector;
struct waitable_access;

/**
 * @brief Caller's instruction for whether signalling should trigger a
 *        reschedule on the *local* core.
 *
 * Namespace scope rather than nested in waitable, because the objects that
 * signal are no longer all waitables.
 */
enum class reschedule_policy
{
   automatic, ///< Conditionally trigger a local reschedule if a better thread is made ready (preferred)
   never,     ///< Never trigger a local reschedule
   always,    ///< Always trigger a local reschedule
};

/**
 * @brief Intrusive priority-ordered list of waiting threads.
 *
 * Owns park-list mechanics and the commit-under-lock discipline that makes a
 * barge-free handoff possible. Owns nothing about what is being waited FOR, so
 * both waitable and the mutex compose one rather than sharing a base.
 *
 * NOT API, despite appearing in a public header. A kernel that does not
 * allocate needs by-value members, and a by-value member needs a complete type
 * where its owner is declared, so the name has to be here. Being usable is a
 * separate question, and the answer is no: every member including the
 * constructor is private, and friendship is granted only to the kernel types
 * that own a queue. Outside those, the name can be written and nothing else.
 * The compiler enforces that, not a comment.
 *
 * The alternative, opaque std::byte storage, would hide the name at the cost of
 * placement-new in every owner's constructor, which costs waitable and
 * semaphore their constexpr constructors and moves them out of .bss.
 */
class CYROS_PUBLIC wait_queue
{
   /**
    * @brief Per-thread parking record (one per TCB, reused).
    *
    * Threaded into one or more queues while the thread is parked. Nested so it
    * is unreachable to anything that cannot already drive a queue.
    */
   struct wait_node
   {
      thread_control_block* owner{nullptr};
      wait_node*            next {nullptr};

      /* Was this waiter counted in its queue's bridge_count when it armed?
       *
       * Recorded rather than recomputed at departure, because a thread can
       * acquire or release while armed, and a count maintained from a value that
       * moved underneath it would drift. */
      bool counted_as_bridge{false};
   };

   using transfer_fn = function<void(uint32_t), 32, heap_policy::no_heap>;
   using commit_fn   = function<void(thread_control_block*), 32, heap_policy::no_heap>;
   static constexpr std::uint8_t no_waiter = 0xFF;

   constexpr wait_queue() noexcept = default;

   wait_queue(wait_queue&&)                 = delete;
   wait_queue(wait_queue const&)            = delete;
   wait_queue& operator=(wait_queue&&)      = delete;
   wait_queue& operator=(wait_queue const&) = delete;

   void arm   (wait_node& n) noexcept;
   bool disarm(wait_node& n) noexcept;

   void wake_one(reschedule_policy) noexcept;
   void wake_all(reschedule_policy) noexcept;
   bool wake_one_and_transfer(transfer_fn const&, reschedule_policy) noexcept;
   bool wake_one_and_commit(commit_fn const& commit, reschedule_policy policy) noexcept;

   [[nodiscard]] bool empty() const noexcept;

   /**
    * @brief Best waiter urgency on this queue, which is what its holder inherits.
    *
    * Fast path is one atomic load. top_priority caches the minimum BASE priority
    * of the waiters, and base never changes, so that cache has only immutable
    * inputs and can never go stale. No re-slotting, ever.
    *
    * The slow path exists because a waiter that itself HOLDS something can be
    * more urgent than its base: that is the transitive chain. Only such a waiter
    * can be, so the fold runs over the bridges alone rather than the whole queue,
    * and bridge_mask is zero in the overwhelming majority of queues.
    *
    * @param depth Remaining recursion budget. A wait-for cycle means the
    *        application has deadlocked; the budget stops the kernel following it
    *        forever and the answer is then conservative rather than wrong.
    */
   [[nodiscard]] std::uint8_t top(unsigned depth) const noexcept;

   void link  (wait_node&) noexcept;
   bool unlink(wait_node&) noexcept;
   void refresh_top()      noexcept;

   spinlock   lock;
   wait_node* head{nullptr}; // base-priority-ordered, best at head
   std::atomic<std::uint8_t> top_priority{no_waiter};

   /* Waiters on this queue that themselves hold a pi_waitable, i.e. exactly the
    * waiters whose urgency can be lower than their base priority. Tracking them
    * separately keeps the transitive fold off the common path: a queue with no
    * bridges answers from top_priority alone.
    *
    * Only a COUNT is kept, not the bridges themselves. Every waitable embeds a
    * queue and every TCB embeds a waitable (its join), so anything stored per
    * queue is paid for by every thread in the system; an array of four pointers
    * here cost 40 bytes on each and blew the TCB budget immediately.
    *
    * The fold therefore snapshots the bridges under the lock, releases it, and
    * only then recurses. That is what keeps queue locks un-nested: the recursion
    * walks a wait-for graph, and holding a lock across it would deadlock the
    * kernel on an application's deadlock rather than merely observing one.
    *
    * More bridges than the snapshot holds is NOT an error: top() then reports 0,
    * the most urgent value, which over-boosts the holder. That costs a little
    * fairness and never correctness, whereas under-reporting would resurrect the
    * unbounded inversion this design exists to prevent. */
   std::atomic<std::uint8_t> bridge_count{0};

   /** @brief Undo a node's bridge contribution. Caller holds the queue lock. */
   void drop_bridge(wait_node&) noexcept;

   // The only types that may drive a queue.
   friend class waitable;
   friend class pi_waitable; // replaced by the kernel mutex
   friend class waitable_arm_guard;
   friend class wait_node_vector;
   friend struct waitable_access;
   friend std::size_t this_thread::wait_on_any(std::span<waitable_ref>) noexcept;
};

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
   virtual ~waitable();

   waitable(waitable&&) = delete;
   waitable(waitable const&) = delete;
   waitable& operator=(waitable&&) = delete;
   waitable& operator=(waitable const&) = delete;

protected:
   using transfer_fn = wait_queue::transfer_fn;

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
   /// Composed, not inherited. See wait_queue above for why the type is
   /// nameable here and still unusable.
   wait_queue queue;

   friend class wait_node_vector;
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
    * Retires the resource from the caller's held slots, then commits the
    * handover (or the free) under the queue lock. There is no restore step:
    * nothing cached the boost, so urgency stops including this resource the
    * moment it leaves held_slots.
    */
   void pi_release(reschedule_policy policy = reschedule_policy::automatic) noexcept;

   void renounce_if_assigned(thread::id thread_id) noexcept override;

private:
   /* One word, not two. The CAS take, the transferred-recognition test and the
    * donation target all read it, so there is no pair to keep ordered. */
   std::atomic<thread_control_block*> owner{nullptr}; // nullptr when free
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
