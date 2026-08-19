#ifndef CYROS_BASE_MUTEX_ACCESS_HPP
#define CYROS_BASE_MUTEX_ACCESS_HPP

#include <cyros/kernel/base_mutex.hpp>

#include <cstdint>

namespace cyros
{

/* ============================================================================
 * base_mutex_access - lends kernel-internal friendship to the urgency machinery
 *
 * Carries no behaviour. The two operations are methods on base_mutex, where
 * they belong; these forward to them and exist only to be the friend.
 *
 * Why the indirection exists at all. The callers are the urgency fold and the
 * prompt walk in kernel.cpp, and neither can be friended directly. The fold has
 * internal linkage, so a friend declaration naming it in a header would name a
 * different entity in every translation unit. Friending the enclosing namespace
 * is not a thing C++ offers, and friending the functions by qualified name would
 * force their declarations into the PUBLIC header, which is the leak this type
 * exists to avoid. A friend STRUCT declaration is self-contained, so it is the
 * one form that costs the public header a single line and discloses nothing.
 *
 * Keep the surface minimal. Every entry here widens what any kernel-internal
 * code can do to a mutex, so adding one deserves the scrutiny of adding a
 * friend, which is exactly what it is.
 * ========================================================================= */
struct base_mutex_access
{
   [[nodiscard]] static std::uint8_t urgency_contribution(base_mutex const& m, unsigned depth) noexcept
   {
      return m.urgency_contribution(depth);
   }

   [[nodiscard]] static thread_control_block* holder_of(base_mutex const& m) noexcept
   {
      return m.holder();
   }
};

} // namespace cyros

#endif // CYROS_BASE_MUTEX_ACCESS_HPP
