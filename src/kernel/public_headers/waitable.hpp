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

class waitable;
class base_mutex;
class waitable_arm_guard;
class wait_node_vector;
struct waitable_access;

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

/**
 * @brief Intrusive priority-ordered list of waiting threads.
 *
 * Owns park-list mechanics and the commit-under-lock discipline that makes a
 * barge-free handoff possible. Owns nothing about what is being waited FOR, so
 * both waitable and the mutex compose one rather than sharing a base.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
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

   /**
    * @brief Undo a node's bridge contribution. Caller holds the queue lock.
    */
   void drop_bridge(wait_node&) noexcept;

   // The only types that may drive a queue.
   friend class waitable;
   friend class base_mutex;
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


} // namespace cyros

#endif // CYROS_WAITABLE_HPP
