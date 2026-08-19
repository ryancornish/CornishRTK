#include <cyros/sync/semaphore.hpp>

namespace cyros::sync
{

[[nodiscard]] std::size_t semaphore::peek() const noexcept
{
   return counter.load(std::memory_order::relaxed);
}

void semaphore::release(std::size_t n) noexcept
{
   counter.fetch_add(n, std::memory_order_release);
   for (std::size_t i = 0; i < n; ++i) {
      wake_one();
   }
}

void semaphore::acquire() noexcept
{
   this_thread::wait_on(*this);
}

[[nodiscard]] bool semaphore::try_acquire() noexcept
{
   auto current = counter.load(std::memory_order_relaxed);
   // Another thread may change counter after we have loaded it.
   // If so, then the compare-exchange will fail and we try again.
   while (current > 0) {
      if (counter.compare_exchange_weak(current, current - 1,
                                        std::memory_order_acquire,
                                        std::memory_order_relaxed)) {
         return true;
      }
   }
   return false;
}

bool semaphore::try_satisfy() noexcept
{
   return try_acquire();
}

}  // namespace cyros::sync
