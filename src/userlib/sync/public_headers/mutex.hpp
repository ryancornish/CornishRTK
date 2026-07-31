#ifndef CYROS_MUTEX_HPP
#define CYROS_MUTEX_HPP

#include <cyros/kernel/waitable.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/kernel/visibility.hpp>

namespace cyros::time { struct time_point; struct duration; }

namespace cyros::sync
{

class CYROS_PUBLIC mutex : public pi_waitable
{
public:
   void unlock() noexcept;

   void lock() noexcept;

   [[nodiscard]] bool try_lock() noexcept;

   [[nodiscard]] bool try_lock_for(time::time_point tp) noexcept;

   [[nodiscard]] bool try_lock_until(time::duration d) noexcept;

protected:
   bool wait_condition(thread&) noexcept override;
};

}  // namespace cyros::sync

namespace cyros { using sync::mutex; }

#endif // CYROS_MUTEX_HPP
