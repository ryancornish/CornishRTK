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
         waitables[i].get().queue.arm(nodes[i]);
      }
   }

   ~waitable_arm_guard()
   {
      for (std::size_t i = 0; i < waitables.size(); ++i) {
         // Return value deliberately ignored. Leaving a queue without acquiring
         // lowers its best-waiter priority, so the holder becomes LESS urgent.
         // Nothing to do: urgency is folded from top() at the point of use, and
         // observing the drop late only means the holder ran at its old urgency
         // slightly longer, which is safe. A boost needs a prompt, a de-boost
         // does not.
         waitables[i].get().queue.disarm(nodes[i]);
      }
   }

   waitable_arm_guard(waitable_arm_guard&&) = delete;
   waitable_arm_guard(waitable_arm_guard const&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard&&) = delete;
   waitable_arm_guard& operator=(waitable_arm_guard const&) = delete;
};

} // namespace cyros

#endif // CYROS_WAITABLE_UTILITIES_VECTOR_HPP
