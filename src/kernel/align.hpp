#ifndef CYROS_ALIGN_HPP
#define CYROS_ALIGN_HPP

#include <cstdint>

namespace cyros
{

/**
 * @brief Aligns a value down to the nearest power-of-two boundary.
 *
 * Example:
 * @code
 * align_down(15, 8); // Returns 8
 * align_down(16, 8); // Returns 16 (already aligned)
 * align_down(17, 8); // Returns 16
 * @endcode
 *
 * @param v Value to align.
 * @param a Alignment boundary (must be a power of two).
 * @return The downward aligned value.
 */
constexpr std::uintptr_t align_down(std::uintptr_t v, std::size_t a)
{
   return v & ~(static_cast<std::uintptr_t>(a) - 1);
}

/**
 * @brief Aligns a value up to the nearest power-of-two boundary.
 *
 * Example:
 * @code
 * align_up(15, 8);   // Returns 16
 * align_up(16, 8);   // Returns 16 (already aligned)
 * align_up(17, 8);   // Returns 24
 * @endcode
 *
 * @param v Value to align.
 * @param a Alignment boundary (must be a power of two).
 * @return The upward aligned value.
 */
constexpr std::uintptr_t align_up(std::uintptr_t v, std::size_t a)
{
   return (v + (a - 1)) & ~(static_cast<std::uintptr_t>(a) - 1);
}

} // namespace cyros

#endif // CYROS_ALIGN_HPP
