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
 * @brief The priority-inheritance state of a base_mutex's wait_queue.
 *
 * Owned by the base_mutex and passed through the wait_queue API to be
 * consumend by wait_queue::top().
 *
 * Owns a 1-1 relationship of priority-inheritance details with a corresponding
 * wait_queue.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC inheritance_cache
{
   static constexpr std::uint8_t no_waiter = 0xFF;

   /**
    * @brief Minimum base-priority of all waiters in the wait_queue
    *
    * Base-priorities are an immutable property of the thread, so this cache
    * won't go stale.
    */
   std::atomic<std::uint8_t> top_priority{no_waiter};

   /**
    * @brief A 'sticky' hint that indicates there _may_ be a waiter in wait_queue
    *        that itself holds another base_mutex (making it a bridge)
    *
    * Sticky in the sense that it is set when a holder arms on the wait_queue,
    * and cleared only when the queue completely empties of waiters.
    *
    * If this is **false**, then there are certainly no bridges in the queue and
    * we need not look (fast path).
    * If this is **true**, then there _may_ be one or more bridges in the queue and
    * we walk the queue to find them.
    *
    * The hint is thus an optimisation that errs only toward 'looking-for', never toward 'missing-a'
    */
   std::atomic<bool> may_have_bridges{false};

   friend class wait_queue;
};

/**
 * @brief Intrusive priority-ordered list of waiting threads.
 *
 * Owns park-list mechanics and the commit-under-lock discipline that makes a
 * barge-free handoff possible.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC wait_queue
{
   /**
    * @brief Per-thread parking record (one per TCB, reused).
    *
    * Threaded into one or more queues while the thread is parked.
    */
   struct wait_node
   {
      thread_control_block* owner{nullptr};
      wait_node*            next {nullptr};
   };

   using transfer_fn = function<void(thread::id), 32, heap_policy::no_heap>;
   using commit_fn   = function<void(thread_control_block*), 32, heap_policy::no_heap>;
   constexpr wait_queue() noexcept = default;

   wait_queue(wait_queue&&)                 = delete;
   wait_queue(wait_queue const&)            = delete;
   wait_queue& operator=(wait_queue&&)      = delete;
   wait_queue& operator=(wait_queue const&) = delete;

   /**
    * @brief Add/remove a waiter onto the queue
    *
    * Generic waitable variant - no inheritance cache
    */
   void arm   (wait_node& node) noexcept { arm(node, nullptr); }
   void disarm(wait_node& node) noexcept { disarm(node, nullptr); }

   void wake_one(reschedule_policy policy) noexcept { wake_one(policy, nullptr); }
   void wake_all(reschedule_policy policy) noexcept { wake_all(policy, nullptr); }
   bool wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy) noexcept
   {
      return wake_one_and_transfer(transfer, policy, nullptr);
   }

   /**
    * @brief Add/remove a waiter onto the queue
    *
    * base_mutex variant - permits an inheritance cache
    */
   void arm   (wait_node& node, inheritance_cache* pi) noexcept;
   void disarm(wait_node& node, inheritance_cache* pi) noexcept;

   void wake_one(reschedule_policy policy, inheritance_cache* pi) noexcept;
   void wake_all(reschedule_policy policy, inheritance_cache* pi) noexcept;
   bool wake_one_and_transfer(transfer_fn const& transfer, reschedule_policy policy, inheritance_cache* pi) noexcept;
   bool wake_one_and_commit(commit_fn const& commit, reschedule_policy policy, inheritance_cache* pi) noexcept;

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
    * and may_have_bridges is false for the overwhelming majority of queues.
    *
    * @param resource_owner The owner word of the resource this queue backs.
    *        Read UNDER the queue lock and excluded from the bridge fold: an
    *        acquirer is briefly both owner and armed waiter in its own queue
    *        (the window between the winning CAS and its disarm), and recursing
    *        into it from its own queue is a self-cycle that exhausts the depth
    *        budget and answers 0. Skipping the owner makes that cycle
    *        unrepresentable at every recursion depth rather than merely brief.
    *
    *        The read must happen inside the lock, not be passed in as a value.
    *        A handover commits the owner under this same lock after unlinking
    *        the chosen waiter, so an under-lock read is exactly consistent
    *        with the queue contents: a thread observed both armed and owning
    *        is the acquire-window self-cycle and nothing else. A value read
    *        before the lock can go stale against exactly that handover, and
    *        the ex-owner can re-arm as a legitimate bridge behind it, which
    *        the stale exclusion would then skip, an UNDER-report.
    *
    *        The lock-free CAS take is the one owner transition outside this
    *        lock, and racing it is UNREACHABLE rather than merely benign. A
    *        fold arrives at a queue only through
    *        waitable_access::urgency_contribution, whose sole caller walks a
    *        thread's held_slots, and a mutex is filed
    *        there only by claim_slot, called either after the CAS that made the
    *        owner or from the handover commit under this lock. So anything able
    *        to consult this queue already observes the owner word set. Stated
    *        as unreachable on purpose: calling it "the safe over-boost
    *        direction" would repeat the mistake this parameter exists to fix,
    *        since a fabricated 0 is unsafe for a RUNNING thread and the
    *        acquirer here is running. Principle 8.
    * @param depth Remaining recursion budget. A wait-for cycle means the
    *        application has deadlocked; the budget stops the kernel following it
    *        forever and the answer is then conservative rather than wrong.
    */
   [[nodiscard]] std::uint8_t top(std::atomic<thread_control_block*> const& resource_owner,
                                  unsigned depth,
                                  inheritance_cache const& pi) const noexcept;

   void link  (wait_node&) noexcept;
   bool unlink(wait_node&) noexcept;
   void refresh_top(inheritance_cache* pi) noexcept;

   spinlock   lock;
   wait_node* head{nullptr}; // base-priority-ordered, best at head

   // The only types that may drive a queue.
   friend class waitable;
   friend class base_mutex;
   friend class pi_wait_queue;
   friend class waitable_arm_guard;
   friend class wait_node_vector;
   friend struct waitable_access;
   friend std::size_t this_thread::wait_on_any(std::span<waitable_ref>) noexcept;
};

/**
 * @brief A wait queue owned by a priority-inheritance resource.
 *
 * Nothing but composition: the queue that parks the waiters, plus the
 * inheritance state only base_mutex ever reads. Each forwarder supplies the
 * cache, so a resource holding one of these cannot forget its own bookkeeping,
 * and a plain waitable holding a bare wait_queue cannot accidentally pay for it.
 *
 * @note Private implementation detail of the kernel. Not a user-facing type!
 */
class CYROS_PUBLIC pi_wait_queue
{
   using wait_node = wait_queue::wait_node;
   using commit_fn = wait_queue::commit_fn;

   void arm   (wait_node& node) noexcept { queue.arm(node, &pi); }
   void disarm(wait_node& node) noexcept { queue.disarm(node, &pi); }

   [[nodiscard]] bool empty() const noexcept { return queue.empty(); }

   bool wake_one_and_commit(commit_fn const& commit, reschedule_policy policy) noexcept
   {
      return queue.wake_one_and_commit(commit, policy, &pi);
   }

   [[nodiscard]] std::uint8_t top(std::atomic<thread_control_block*> const& resource_owner,
                                  unsigned depth) const noexcept
   {
      return queue.top(resource_owner, depth, pi);
   }

   wait_queue        queue;
   inheritance_cache pi;

   friend struct waitable_access;
   friend class base_mutex;
};


/* ============================================================================
 * waitable - kernel base class for blockable objects
 * ----------------------------------------------------------------------------
 * Inherit from waitable to make your object something threads can block ON
 * (semaphore, event, thread-termination, a future alarm). The base owns a
 * private wait_queue and exposes to the derived class:
 *
 *   try_satisfy()           virtual. Attempt to satisfy the caller without
 *                           blocking, consuming the resource if that is what
 *                           satisfaction means. wait_on_any polls it between
 *                           arm and park, which is the two-phase block that
 *                           closes the lost-wakeup window. Runs in the calling
 *                           thread's context, so an implementation that needs
 *                           the caller's identity uses this_thread::id().
 *   wake_one / wake_all     signal waiters. ISR-safe.
 *   wake_one_and_transfer   barge-free handoff, see the protocol note below.
 *
 * Threads wait via this_thread::wait_on(w) or wait_on_any(w0, w1, ...), never
 * through methods on the waitable itself. Use composition instead of
 * inheritance when your object merely uses blocking internally without being a
 * wait target.
 *
 * Barging, and the transfer protocol
 * ----------------------------------
 * waitable is BARGE-PERMITTING by default: a wake does not reserve anything
 * for the woken thread, so a fresh caller may take the resource first and the
 * woken thread re-parks. That is cheap and right for counting semaphores,
 * events and joins.
 *
 * Barge-free handoff, where a release commits the resource to the chosen
 * waiter under the queue lock so nothing can interpose, is available through
 * wake_one_and_transfer plus a try_satisfy that recognises ownership by id,
 * plus renounce_if_assigned for the group-wait interaction. This CANNOT be
 * built outside the kernel from wake_one: choosing the best waiter and
 * committing to it must happen under the queue lock, which nothing else can
 * reach. That is why the surface exists here.
 *
 * STATUS: no in-tree primitive currently derives a transfer-shaped waitable.
 * The kernel mutex needs barge-free handoff WITH priority inheritance and gets
 * both from base_mutex, which composes the same wait_queue directly and is
 * deliberately not a waitable. This surface is the path for a barge-free
 * primitive WITHOUT inheritance (a strict-fairness semaphore, a mailbox that
 * hands its datum to exactly one waiter). test_transfer_waitables is its
 * conformance test and is what keeps the dual-satisfaction contract honest.
 *
 * Spurious wakeups
 * ----------------
 * A woken thread is not guaranteed its condition holds: the resource may have
 * been consumed before it runs. wait_on_any re-checks and re-parks
 * transparently, the same mandatory loop as around pthread_cond_wait.
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
    * @brief Attempt to satisfy the calling thread without blocking.
    *
    * Consume the resource if that is what satisfaction means (a semaphore
    * decrements, a transfer-shaped primitive recognises ownership by
    * this_thread::id()). Runs in the calling thread's context with no queue
    * lock held.
    *
    * @return true when the caller is satisfied and will not block.
    */
   virtual bool try_satisfy() noexcept = 0;

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
    * derived hook must recognise ownership by id in addition to
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
   bool try_satisfy() noexcept override
   {
      return true;
   }
};


} // namespace cyros

#endif // CYROS_WAITABLE_HPP
