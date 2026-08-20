/**
 * @file diagnostics.cpp
 * @brief The blocking-chain query and its deadlock confirmation.
 *
 * The whole walk is lock-free: blocked_on, owner and state are the same
 * atomics the urgency fold reads, published for exactly this kind of remote
 * consumption. Taking no locks is what makes the query safe to call from
 * anywhere and impossible to deadlock WITH.
 */

#include <cyros/kernel/diagnostics.hpp>

#include "base_mutex_access.hpp"
#include "threading_subsystem.hpp"

namespace cyros
{

/* Attorney for the one private read this file needs. Defined here and nowhere
 * else, so the friend line in thread.hpp grants exactly this file access. */
struct diagnostics_access
{
   [[nodiscard]] static thread_control_block const* tcb_of(thread const& t) noexcept
   {
      return t.tcb;
   }
};

namespace diagnostics
{

namespace
{

/* One hop: the holder of the mutex @p t is blocked on, or nullptr when t is
 * not blocked on a mutex, the mutex has just been freed, or the mutex has
 * already been handed to t ITSELF.
 *
 * That last case is the load-bearing one. Between a handover committing the
 * owner and the new owner running to clear its blocked_on, the thread reads
 * as blocked on a mutex it owns, and on a churning system that window is the
 * OVERWHELMING majority of observed edges (11k of 15k in the suite's churn
 * census). It can even read state == blocked, because the owner store lands
 * before the wake flips the state, so no re-read of the node catches it. It
 * is excluded structurally instead: a real self-wait cannot exist, the
 * kernel asserts an owner arming on its own queue away, so owner == t always
 * means "resolving", never "waiting". */
[[nodiscard]] thread_control_block const* next_holder(thread_control_block const& t) noexcept
{
   auto const* m = t.blocked_on.load(std::memory_order_acquire);
   if (m == nullptr) {
      return nullptr;
   }
   auto* holder = base_mutex_access::holder_of(*m);
   if (holder == &t) {
      return nullptr;
   }
   return holder;
}

/* A real deadlock is frozen: every thread on the cycle is blocked and its
 * blocked_on and the owner of that mutex can never change again. So a cycle
 * candidate is confirmed by walking it once more and requiring every node
 * blocked and every edge identical. A phantom cycle, assembled from reads of
 * a graph that moved mid-walk, fails this because the edges it was stitched
 * from are already gone. One re-read cannot be fooled by a cycle that exists
 * across time but never at an instant, because a node whose edge changed
 * since the first walk no longer links where the candidate said it did. */
[[nodiscard]] bool confirm_cycle(thread_control_block const* const* nodes,
                                 std::size_t first, std::size_t count) noexcept
{
   for (std::size_t i = first; i < count; ++i) {
      auto const* node = nodes[i];
      if (node->state != thread_state::blocked) {
         return false;
      }
      auto const* expected = (i + 1 < count) ? nodes[i + 1] : nodes[first];
      if (next_holder(*node) != expected) {
         return false;
      }
   }
   return true;
}

} // namespace

blocking_chain blocking_chain_of(thread const& t) noexcept
{
   blocking_chain out;

   auto const* start = diagnostics_access::tcb_of(t);
   if (start == nullptr) {
      return out; // Moved-from or never-started handle
   }

   // visited[0] is the queried thread, visited[1..] the successive holders.
   std::array<thread_control_block const*, blocking_chain::max_links + 1> visited{};
   visited[0] = start;
   std::size_t count = 1;

   auto const* current = start;
   while (out.length < blocking_chain::max_links) {
      auto const* holder = next_holder(*current);
      if (holder == nullptr) {
         return out;
      }

      for (std::size_t i = 0; i < count; ++i) {
         if (visited[i] == holder) {
            out.deadlocked = confirm_cycle(visited.data(), i, count);
            return out;
         }
      }

      out.holders[out.length++] = holder->id;
      visited[count++] = holder;
      current = holder;
   }
   return out;
}

} // namespace diagnostics
} // namespace cyros
