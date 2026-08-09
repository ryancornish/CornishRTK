#ifndef CYROS_THREAD_HPP
#define CYROS_THREAD_HPP

#include <cyros/port/port_traits.h>
#include <cyros/kernel/function.hpp>
#include <cyros/kernel/visibility.hpp>

#include <cstdint>
#include <functional>
#include <span>

namespace cyros
{

using waitable_ref = std::reference_wrapper<class waitable>;

/**
 * @brief Core affinity mask
 *
 * Bit flags indicating which cores a thread can run on.
 * Use bitwise OR to combine cores: core0 | core1
 */
struct core_affinity
{
   std::uint32_t mask;
   constexpr explicit core_affinity(std::uint32_t m) : mask(m) {}
   constexpr explicit operator std::uint32_t() const { return mask; }
   constexpr core_affinity operator|(core_affinity rhs) const { return core_affinity{mask | rhs.mask}; }
   constexpr core_affinity operator&(core_affinity rhs) const { return core_affinity{mask & rhs.mask}; }
   [[nodiscard]] constexpr bool allows(uint32_t core_id) const noexcept { return (mask & (1u << core_id)) != 0; }
   [[nodiscard]] constexpr static core_affinity from_id(std::uint32_t core_id) { return core_affinity{1u << core_id}; }
};
// Predefined core masks
inline constexpr core_affinity core0 = core_affinity{0x01};
inline constexpr core_affinity core1 = core_affinity{0x02};
inline constexpr core_affinity core2 = core_affinity{0x04};
inline constexpr core_affinity core3 = core_affinity{0x08};
inline constexpr core_affinity any_core = core_affinity{0xFFFFFFFF};


/**
 * @brief Joinable Cyros thread handle.
 *
 * Owns a running kernel thread. The thread's TCB is constructed inside the user-provided
 * stack buffer, so both the @c thread object and the stack buffer must outlive the thread.
 *
 * The destructor asserts the thread is terminated (i.e. no implicit detach).
 */
class CYROS_PUBLIC thread
{
public:
   using id = std::uint32_t;
   using entry_fn = function<void(), 48, heap_policy::no_heap>;

   struct priority
   {
      std::uint8_t val;
      constexpr priority(std::uint8_t v) : val(v) {}     // Intentionally implicit
      constexpr operator uint8_t() const { return val; } // Intentionally implicit
   };


   /**
    * @brief Create empty thread handle.
    * A registered thread can be moved into this.
    */
   constexpr thread() = default;
   /**
    * @brief Create and register a new thread.
    * @param entry thread entry function.
    * @param stack User-owned stack buffer (must remain valid until termination).
    * @param priority Initial priority.
    * @param affinity Core affinity (defaults to any_core).
    */
   thread(entry_fn&& entry, std::span<std::byte> stack, priority priority, core_affinity affinity = any_core);
   ~thread();
   thread(thread&&) noexcept;
   thread& operator=(thread&&) noexcept;
   thread(thread const&)            = delete;
   thread& operator=(thread const&) = delete;

   /**
    * @brief Get thread ID
    * @return Unique thread identifier
    */
   [[nodiscard]] id get_id() const noexcept;

   /**
    * @brief Get thread priority
    * @return Current effective priority (base + inherited)
    */
   [[nodiscard]] priority get_priority() const noexcept;

   /**
    * @brief Wait for thread to exit
    *
    * Blocks until this thread terminates.
    * Can only be called once per thread.
    */
   void join() noexcept;

private:
   struct thread_control_block* tcb{nullptr};

public:
   /**
    * @brief Budget for the kernel-private part of the TCB, everything except
    *        the port context.
    *
    * This is a rough overestimation. The actual size cannot be a compile-time constant
    * because it varies by port, and on one port it can still vary with the hardware,
    * (so not expressible via port_traits). Instead this catches if tcb grows past the estimation.
    * A generous fixed budget plus a static_assert is the working compromise.
    */
   /* Bump if sizeof(thread_control_block) grows.
    *
    * 256 was EXACTLY full before this work. Since then, per TCB:
    *   + intake_next   (8)  the cross-core transport
    *   + holder_next   (8)  the ready-holder list the pick folds over
    *   - effective_priority the cache derived urgency replaced
    *   - prev          (8)  the ready queue is singly linked now that nothing
    *                        re-keys a thread's position
    *   - active_waits  (8)  a pointer nobody read once the recompute that
    *                        re-slotted armed nodes was deleted
    *
    * MEASURED 2026-08-09, do not re-derive this by counting fields. The TCB is
    * 64-byte aligned and sizeof() lands on 3840 with the port context included,
    * which is exactly the budget. There is NO trailing slack: adding a single
    * byte anywhere pushes sizeof to 3904. Removing active_waits changed sizeof
    * by nothing at all, because it sat in an interior padding hole rather than
    * at the end, so the field count and the byte count move independently. Any
    * future claim about headroom has to come from a probe, not arithmetic.
    *
    * Raising this is allowed and is a judgement call, not a wall. It feeds
    * min_stack_size and therefore every user's stack, so the cost is real and
    * belongs in the commit message alongside what the bytes buy. Reclaiming
    * bytes instead means reordering members to collapse the holes, which is a
    * separate exercise nobody has done.
    *
    * Migration does not resurrect the need for prev, provided it is done at the
    * pick (pop, check pinned_core, forward) rather than by removing from the
    * middle of a ready queue. */
   static constexpr std::size_t tcb_size = 256;

   /**
    * @brief Headroom reserved for the first call frame on a fresh stack.
    */
   static constexpr std::size_t min_frame = 4096;

   /**
    * @brief Minimal size (bytes) a stack must be for a thread to execute on
    */
   static constexpr std::size_t min_stack_size = CYROS_PORT_CONTEXT_SIZE + tcb_size + min_frame;
};

namespace this_thread
{

/**
   * @brief Get current thread ID
   */
[[nodiscard]] CYROS_PUBLIC thread::id id();

/**
   * @brief Get current thread (effective) priority
   */
[[nodiscard]] CYROS_PUBLIC thread::priority priority();

/**
   * @brief Exit current thread
   *
   * Marks current thread as terminated. thread never runs again.
   * scheduler switches to next ready thread.
   *
   * Note: If thread entry function returns, this is called automatically.
   */
[[noreturn]] CYROS_PUBLIC void thread_exit();

CYROS_PUBLIC void yield();

[[nodiscard]] CYROS_PUBLIC std::size_t wait_on_any(std::span<waitable_ref> waitables) noexcept;

template<typename... Waitables>
[[nodiscard]] std::size_t wait_on_any(Waitables&... waitables)
{
   static_assert(sizeof...(Waitables) > 0);
   static_assert((std::is_base_of_v<waitable, std::remove_cv_t<Waitables>> && ...));
   std::array<waitable_ref, sizeof...(Waitables)> inline_array{ std::ref(waitables)... };
   return wait_on_any(std::span<waitable_ref>{inline_array});
}

inline void wait_on(waitable& w)
{
   // single-source call always returns 0
   (void)wait_on_any(w);
}


}  // namespace this_thread

} // namespace cyros

#endif // CYROS_THREAD_HPP
