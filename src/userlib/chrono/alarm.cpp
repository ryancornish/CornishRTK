/**
 * @file alarm.cpp
 * @brief The alarm waitable and the sleeps built on it.
 *
 * Everything here is public kernel and time surface: derive waitable, wake
 * from the timer callback, block through this_thread. That is the feature's
 * point as much as its function.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/port/port.h>

namespace cyros::chrono
{

alarm::~alarm()
{
   disarm();
}

/* Runs in the driver's callback context: a timer ISR on the hardware-shaped
 * drivers, the pump context under simulation. Two statements, both ISR-legal,
 * and disarm's spin is sized to exactly this body. */
void alarm::on_timer(void* self) noexcept
{
   auto& a = *static_cast<alarm*>(self);
   a.fired.store(true, std::memory_order_release);
   a.wake_all();
}

bool alarm::pending() const noexcept
{
   return timer.id != 0 && !fired.load(std::memory_order_acquire);
}

void alarm::arm_at(time::time_point tp) noexcept
{
   CYROS_ASSERT(!pending()); // Re-arming a pending alarm is a cancel-then-arm at the call site

   fired.store(false, std::memory_order_relaxed);
   timer = time::schedule_at(tp, &on_timer, this);
   CYROS_ASSERT(timer.id != 0); // Timer table full, a config-sized limit
}

void alarm::arm_in(time::duration d) noexcept
{
   arm_at(time::now() + d);
}

void alarm::disarm() noexcept
{
   if (timer.id != 0 && !time::cancel(timer)) {
      /* The cancel missed, so the callback ran or is running. On the periodic
       * and tickless drivers it ran: the callback executes in the arming
       * core's timer ISR and this thread is on that core, so an ISR that
       * started has finished. Under simulation the pump fires callbacks
       * outside its lock from whichever context advances time, so wait out
       * the two-statement body before letting the caller destroy us. */
      while (!fired.load(std::memory_order_acquire)) {
         cyros_port_cpu_relax();
      }
   }
   timer = {};
   fired.store(false, std::memory_order_relaxed);
}

bool alarm::expired() const noexcept
{
   return fired.load(std::memory_order_acquire);
}

bool alarm::try_satisfy() noexcept
{
   return fired.load(std::memory_order_acquire);
}

void sleep_until(time::time_point tp) noexcept
{
   alarm a;
   a.arm_at(tp);
   this_thread::wait_on(a);
   // The alarm has fired by construction of the return, so the destructor's
   // disarm is bookkeeping, not a cancellation.
}

void sleep_for(time::duration d) noexcept
{
   if (d.value == 0) {
      return;
   }
   sleep_until(time::now() + d);
}

} // namespace cyros::chrono
