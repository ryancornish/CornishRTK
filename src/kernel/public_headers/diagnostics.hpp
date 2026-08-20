#ifndef CYROS_DIAGNOSTICS_HPP
#define CYROS_DIAGNOSTICS_HPP

#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/visibility.hpp>

#include <array>
#include <cstddef>

namespace cyros::diagnostics
{

/**
 * @brief The chain of holders a thread is transitively blocked behind.
 *
 * holders[0] owns the mutex the queried thread is blocked on, holders[1] owns
 * the mutex holders[0] is blocked on, and so on. length is how many entries
 * are valid. A thread that is not blocked on a mutex reports length 0.
 *
 * deadlocked means the walk returned to a thread already on the chain and the
 * cycle re-read identically: a wait-for cycle. The hop that closes the cycle
 * is not repeated in holders, so a two-thread deadlock queried from inside it
 * reports length 1 and deadlocked true. A REAL deadlock is permanent,
 * its threads can never run again, so a confirmed cycle stays confirmed and
 * re-querying gives the same answer forever. The confirmation re-read exists
 * because the graph moves under the walk: a handover between two reads can
 * present a cycle that never existed at any one instant, and a single-pass
 * report would call that a deadlock. See the .cpp for the exact discipline.
 *
 * Advisory by design. The snapshot is taken lock-free from the same atomics
 * the urgency fold reads, so it can be stale in the benign direction (a chain
 * that just resolved may still be reported). Use it for debugging and
 * watchdog-style health checks, never as an input to scheduling decisions.
 */
struct blocking_chain
{
   /// Mirrors the urgency fold's recursion bound. A longer real chain exists
   /// only in an application already past what inheritance can bound.
   static constexpr std::size_t max_links = 8;

   std::array<thread::id, max_links> holders{};
   std::size_t length{0};
   bool deadlocked{false};
};

/**
 * @brief Who is @p t transitively blocked behind, right now.
 *
 * Callable from any thread context with no locks taken. The queried thread
 * must outlive the call, which its owner (the holder of the thread handle)
 * is already promising by holding the handle.
 */
[[nodiscard]] CYROS_PUBLIC blocking_chain blocking_chain_of(thread const& t) noexcept;

} // namespace cyros::diagnostics

#endif // CYROS_DIAGNOSTICS_HPP
