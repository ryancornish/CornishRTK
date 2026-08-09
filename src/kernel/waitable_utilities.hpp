#ifndef CYROS_WAITABLE_UTILITIES_VECTOR_HPP
#define CYROS_WAITABLE_UTILITIES_VECTOR_HPP

#include <cyros/config/config.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/port/port.h>

#include "thread_action.hpp"
#include "threading_subsystem.hpp"

#include <array>

namespace cyros
{

/* ============================================================================
 * waitable_access - attorney for the priority-inheritance walk
 *
 * The urgency fold needs one narrow operation on pi_waitable, and
 * pi_waitable internals. Granting friendship to the function itself would
 * force its declaration into the public header (a friend FUNCTION must be
 * visibly declared to be callable, a friend STRUCT declaration is
 * self-contained), and granting friendship to all of thread_action would
 * bleed access far wider than the walk needs. This attorney is the narrow
 * waist: the public header carries only "friend struct waitable_access", the
 * capability lives here in a kernel-internal header, and the walk consumes
 * these public statics with no friendship of its own, which also frees it to
 * be decomposed into ordinary helpers.
 *
 * Keep this surface minimal on purpose: every method added here widens what
 * ANY kernel-internal code can do to a waitable, so a new entry needs the
 * same scrutiny a new friend would.
 * ========================================================================= */
struct waitable_access
{
   /// Best queued waiter's urgency for a held PI resource. Lock-free at every
   /// depth: see thread_action::urgency_at.
   [[nodiscard]] static std::uint8_t queue_top(pi_waitable const& w, unsigned depth) noexcept
   {
      return w.queue.top(depth);
   }

   /// Re-order an armed node after its owner's priority changed.
   /// @return true when the queue's best-waiter priority changed.
   [[nodiscard]] static bool reslot(waitable& w, wait_queue::wait_node& node) noexcept
   {
      return w.queue.reslot(node);
   }

   /// The thread inheriting from this waitable's waiters, nullptr for
   /// ownerless waitables. expected_id guards against TCB recycling.
   [[nodiscard]] static thread_control_block* donation_target(waitable& w, thread::id& expected_id) noexcept
   {
      return w.donation_target(expected_id);
   }
};

class wait_node_vector
{
private:
   using wait_node = wait_queue::wait_node;

   std::array<wait_node, config::max_wait_nodes> store{};
   std::size_t count = 0;

public:
   constexpr wait_node_vector() = default;
   constexpr wait_node_vector(std::size_t node_count, thread_control_block& tcb)
   {
      for (std::size_t i = 0; i < node_count; ++i) {
         push({
            .owner = &tcb,
            .next = nullptr,
            .source_index = static_cast<uint8_t>(i),
         });
      }
   }

   using iterator = wait_node*;
   using const_iterator = wait_node const*;

   iterator begin()
   {
      return store.data();
   }

   iterator end()
   {
      return store.data() + count;
   }

   [[nodiscard]] bool empty() const noexcept
   {
      return count == 0;
   }

   [[nodiscard]] size_t size() const noexcept
   {
      return count;
   }

   [[nodiscard]] const_iterator begin() const noexcept
   {
      return store.data();
   }

   [[nodiscard]] const_iterator end() const noexcept
   {
     return store.data() + count;
   }

   [[nodiscard]] constexpr size_t capacity() const noexcept
   {
     return store.size();
   }

   wait_node const& operator[](std::size_t index) const noexcept
   {
      CYROS_ASSERT(index < count);
      return store[index];
   }

   wait_node& operator[](std::size_t index) noexcept
   {
      CYROS_ASSERT(index < count);
      return store[index];
   }

   void push(wait_node node)
   {
      CYROS_ASSERT(count < store.size());
      store[count++] = node;
   }
};

class waitable_arm_guard
{
   std::span<waitable_ref> waitables;
   wait_node_vector& nodes;
public:
   waitable_arm_guard(std::span<waitable_ref> waitables, wait_node_vector& nodes)
      : waitables(waitables), nodes(nodes)
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         auto& waitable = waitables[i].get();
         nodes[i].source = &waitable;
         waitable.queue.arm(nodes[i]);
      }
   }

   ~waitable_arm_guard()
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         auto& waitable = waitables[i].get();
         if (!waitable.queue.disarm(nodes[i])) {
            continue;
         }
         // Leaving a queue without acquiring lowers its best-waiter priority,
         // so the holder becomes LESS urgent. Nothing to do: urgency is folded
         // from top() at the point of use, and observing the drop late only
         // means the holder ran at its old urgency slightly longer, which is
         // safe. A boost needs a prompt, a de-boost does not.
      }
   }

   waitable_arm_guard(waitable_arm_guard&&) = delete;
   waitable_arm_guard(waitable_arm_guard const&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard&&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard const&) = delete;
};

// Publish the nodes for priority inheritance: a recompute on this core
// must be able to find and re-slot our armed nodes while we are blocked
// (or preempted mid-block). Registered for the whole attempt and cleared
// before the stack-resident vector dies. Guarded by pi_lock because a
// recompute dereferences the vector under that lock, and destruction
// ordering keeps the final disarm (arm_guard, constructed later inside the
// loop, destroyed earlier) ahead of this deregistration.
class active_wait_registration
{
   thread_control_block& tcb;

public:
   active_wait_registration(thread_control_block& tcb, wait_node_vector* nodes) : tcb(tcb)
   {
      spinlock_guard guard(tcb.pi_lock);
      tcb.active_waits = nodes;
   }
   ~active_wait_registration()
   {
      spinlock_guard guard(tcb.pi_lock);
      tcb.active_waits = nullptr;
   }

   active_wait_registration(active_wait_registration&&) = delete;
   active_wait_registration(active_wait_registration const&) = delete;
   active_wait_registration& operator=(active_wait_registration&&) = delete;
   active_wait_registration& operator=(active_wait_registration const&) = delete;
};

} // namespace cyros

#endif // CYROS_WAITABLE_UTILITIES_VECTOR_HPP
