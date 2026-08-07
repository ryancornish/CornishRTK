#ifndef CYROS_RELAXED_ATOMIC_HPP
#define CYROS_RELAXED_ATOMIC_HPP

#include <atomic>
#include <concepts>

namespace cyros
{

/**
 * @brief std::atomic wrapper that defaults to std::memory_order_relaxed access.
 *
 * This wrapper enforces memory operations to default to relaxed memory ordering.
 * It is designed for single-writer, multi-reader variables where sequential
 * consistency or memory barriers are unnecessary, but standard types are unsafe.
 *
 * @details Although a raw type (like uint32_t) might guarantee atomic, non-tearing
 * modifications at the hardware level on modern CPU architectures, using a plain
 * variable is still a data race according to the C++ memory model. Without an atomic
 * type, the compiler is free to make aggressive optimizations that break cross-core
 * communication. Such optimizations include:
 * - **Register Caching**: The compiler may cache the variable's value in a CPU register
 *   indefinitely inside a loop, preventing reading cores from ever seeing updates
 *   written by the owning core.
 * - **Instruction Reordering**: The compiler may reorder or completely optimize away
 *   reads or writes it deems redundant within a single-threaded context.
 *
 * Wrap variables in this class to signal to the compiler that the memory can be
 * modified by other threads. This forces the compiler to generate actual load/store
 * instructions (preventing register caching) while maintaining maximum performance via
 * relaxed semantics (`std::memory_order_relaxed`).
 */
template <typename T>
class relaxed_atomic
{
   std::atomic<T> value;

public:
   constexpr relaxed_atomic() = default;
   constexpr explicit relaxed_atomic(T initial) : value(initial) {}

   operator T() const noexcept // NOLINT(*-explicit-constructor): Allow implicit loads
   {
      return value.load(std::memory_order_relaxed);
   }

   T operator=(T desired) noexcept // NOLINT(*-unconventional-assign-operator, *-c-copy-assignment-signature): Allow implicit stores
   {
      value.store(desired, std::memory_order_relaxed);
      return desired;
   }

   T load() const noexcept
   {
      return value.load(std::memory_order_relaxed);
   }

   void store(T desired) noexcept
   {
      value.store(desired, std::memory_order_relaxed);
   }

   // Allow explicit casts to integral types (or otherwise convertible)
   template <typename U> requires (std::is_convertible_v<T, U> || (std::is_enum_v<T> && std::integral<U>))
   explicit operator U() const noexcept
   {
      return static_cast<U>(value.load(std::memory_order_relaxed));
   }
};

}  // namespace cyros

#endif // CYROS_RELAXED_ATOMIC_HPP
