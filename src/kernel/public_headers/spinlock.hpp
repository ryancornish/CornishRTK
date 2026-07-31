#ifndef CYROS_SPINLOCK_HPP
#define CYROS_SPINLOCK_HPP

#include <cyros/kernel/core.hpp>

#include <atomic>
#include <cyros/kernel/visibility.hpp>

namespace cyros
{

/**
 * @brief Interrupt-masking spinlock for short critical sections
 *
 * Holding the lock is an interrupt-masking critical section on the calling
 * core: neither a thread switch nor an ISR can interleave with the holder.
 *
 * The cost of the guarantee is a latency contract: the system's worst-case
 * interrupt latency includes the longest hold plus any contention spin, so
 * sections must stay tiny (i.e. a few dozen instructions). For longer critical
 * sections, use a mutex.
 *
 * Usage:
 *   spinlock lock;
 *   lock.lock();
 *   // ... critical section ...
 *   lock.unlock();
 *
 * Or with RAII:
 *   spinlock lock;
 *   {
 *       spinlock_guard guard(lock);
 *       // ... critical section ...
 *   } // Automatically unlocked
 */
class CYROS_PUBLIC spinlock
{
public:
   constexpr spinlock() : flag(ATOMIC_FLAG_INIT) {}

   ~spinlock() = default;

   spinlock(spinlock const&)            = delete;
   spinlock& operator=(spinlock const&) = delete;
   spinlock(spinlock&&)                 = delete;
   spinlock& operator=(spinlock&&)      = delete;

   /**
    * @brief Acquire the spinlock (busy-wait)
    *
    * Blocks until the lock is acquired.
    * Uses CPU hints to reduce power consumption while spinning.
    */
   void lock();

   /**
    * @brief Release the spinlock
    */
   void unlock();

   /**
    * @brief Try to acquire the spinlock without blocking
    * @return true if acquired, false if already locked
    */
   bool try_lock()
   {
      return !flag.test_and_set(std::memory_order_acquire);
   }

   /**
    * @brief Check if the spinlock is currently locked
    * @return true if locked, false if unlocked
    *
    * Note: This is racy and should only be used for debugging/assertions.
    */
   [[nodiscard]] bool is_locked() const
   {
      return flag.test(std::memory_order_relaxed);
   }

private:
   std::atomic_flag flag;
   this_core::critical_token token{};
};

/**
 * @brief RAII guard for spinlocks
 *
 * Automatically acquires lock on construction and releases on destruction.
 */
class CYROS_PUBLIC spinlock_guard
{
public:
   explicit spinlock_guard(spinlock& lock) : lock(lock)
   {
      lock.lock();
   }

   ~spinlock_guard()
   {
      lock.unlock();
   }

   spinlock_guard(spinlock_guard const&)            = delete;
   spinlock_guard& operator=(spinlock_guard const&) = delete;
   spinlock_guard(spinlock_guard&&)                 = delete;
   spinlock_guard& operator=(spinlock_guard&&)      = delete;

private:
   spinlock& lock;
};

} // namespace cyros

#endif // CYROS_SPINLOCK_HPP
