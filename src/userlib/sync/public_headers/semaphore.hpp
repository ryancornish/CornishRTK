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

   /**
    * @brief try_acquire, blocking up to @p d / until @p tp for a token.
    *
    * PROVIDED BY THE CHRONO FEATURE. sync is time-free: these are declared
    * here so the type is complete, and their definitions live in chrono.
    * Calling one without chrono in the package is a link error naming the
    * method, which is the intended diagnosis, not a bug. Merely instantiating
    * a semaphore never references them, so sync alone stays link-clean.
    *
    * A deadline at or before now degrades to plain try_acquire.
    */
   [[nodiscard]] bool try_acquire_for(time::duration d) noexcept;

   [[nodiscard]] bool try_acquire_until(time::time_point tp) noexcept;

protected:
   bool try_satisfy() noexcept override;

private:
   std::atomic<std::size_t> counter;
};

}  // namespace cyros::sync

namespace cyros { using sync::semaphore; }

#endif // CYROS_SEMAPHORE_HPP
