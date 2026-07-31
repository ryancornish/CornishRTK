#ifndef CYROS_SEMAPHORE_HPP
#define CYROS_SEMAPHORE_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/visibility.hpp>

#include <atomic>
#include <cstddef>

namespace cyros::time { struct time_point; struct duration; }

namespace cyros::sync
{

class CYROS_PUBLIC semaphore : public waitable
{
public:
   constexpr explicit semaphore(std::size_t n) : counter(n) {}

   [[nodiscard]] std::size_t peek() const noexcept;

   void release(std::size_t n = 1) noexcept;

   void acquire() noexcept;

   [[nodiscard]] bool try_acquire() noexcept;

   [[nodiscard]] bool try_acquire_for(time::time_point tp) noexcept;

   [[nodiscard]] bool try_acquire_until(time::duration d) noexcept;

protected:
   bool wait_condition(thread&) noexcept override;

private:
   std::atomic<std::size_t> counter;
};

}  // namespace cyros::sync

namespace cyros { using sync::semaphore; }

#endif // CYROS_SEMAPHORE_HPP
