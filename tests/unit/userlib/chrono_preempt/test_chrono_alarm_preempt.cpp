/**
 * @file test_chrono_alarm_preempt.cpp
 * @brief The alarm's real-ISR path: preempt port, periodic driver.
 *
 * The sim suite proves the semantics at exact ticks. This suite proves the
 * mechanism under a real asynchronous timer: the callback runs in a timer ISR
 * that interrupted whatever was running, and the wake it issues crosses back
 * into thread context through the ISR-safe wake path. Assertions are order and
 * at-or-after bounds only. Wall-clock upper bounds are deliberately absent, a
 * sleep that never wakes fails as a suite hang, not as a flaky margin.
 *
 * time::start() enables the CALLING core's tick and schedule_at files into the
 * calling core's timetable, so the most urgent thread starts time before
 * anything arms. Single core, so there is exactly one timetable in play.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/time/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>

using namespace cyros;

namespace
{

constexpr auto frequency = 1'000u; // 1 kHz tick, millisecond granularity

class ChronoAlarmPreempt_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      kernel::initialise();
      time::initialise(frequency);
   }

   void TearDown() override
   {
      time::finalise();
      kernel::finalise();
   }
};

alignas(16) std::array<std::byte, 128 * 1024> s_a;
alignas(16) std::array<std::byte, 128 * 1024> s_b;

} // namespace

TEST_F(ChronoAlarmPreempt_Test, GivenBlockedWaiter_WhenTheAlarmFiresInTheTimerIsr_ThenItWakesAtOrAfterTheDeadline)
{
   uint64_t deadline = 0;
   uint64_t woke_at = 0;
   bool was_expired = false;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm a;
         a.arm_in(time::from_milliseconds(10));
         deadline = time::now().value + 10;

         this_thread::wait_on(a);

         woke_at = time::now().value;
         was_expired = a.expired();
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_TRUE(was_expired);
   EXPECT_GE(woke_at + 1, deadline); // +1 absorbs the arm-vs-deadline read race
}

TEST_F(ChronoAlarmPreempt_Test, GivenSleepFor_WhenRealTimeElapses_ThenAtLeastTheDurationPassed)
{
   uint64_t before = 0;
   uint64_t after = 0;

   thread sleeper(
      [&]{
         time::start();
         before = time::now().value;
         this_thread::sleep_for(time::from_milliseconds(20));
         after = time::now().value;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_GE(after - before, 20u);
}

TEST_F(ChronoAlarmPreempt_Test, GivenTimedWait_WhenAnotherThreadReleasesFirst_ThenTheSemaphoreIndexWins)
{
   sync::semaphore sem(0);
   std::size_t index = 999;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm deadline;
         deadline.arm_in(time::from_milliseconds(500));
         index = this_thread::wait_on_any(sem, deadline);
         deadline.disarm();
      },
      s_a, thread::priority(1)
   );

   thread releaser(
      [&]{
         this_thread::sleep_for(time::from_milliseconds(5));
         sem.release();
      },
      s_b, thread::priority(2)
   );

   kernel::start();

   EXPECT_EQ(index, 0u); // the semaphore, well inside the deadline
}

TEST_F(ChronoAlarmPreempt_Test, GivenTimedWait_WhenNothingReleases_ThenTheAlarmIndexWins)
{
   sync::semaphore sem(0);
   std::size_t index = 999;

   thread waiter(
      [&]{
         time::start();
         chrono::alarm deadline;
         deadline.arm_in(time::from_milliseconds(10));
         index = this_thread::wait_on_any(sem, deadline);
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_EQ(index, 1u); // the deadline
}

TEST_F(ChronoAlarmPreempt_Test, GivenABusySpinner_WhenTheAlarmFires_ThenTheWokenSleeperPreemptsIt)
{
   // The ISR wake must not just ready the sleeper, it must get it RUNNING
   // ahead of a lower-urgency thread that never yields. This is the preempt
   // port earning its name on the alarm path.
   std::atomic<bool> woke{false};
   std::atomic<bool> spinner_saw_wake{false};

   thread sleeper(
      [&]{
         time::start();
         this_thread::sleep_for(time::from_milliseconds(10));
         woke.store(true, std::memory_order_release);
      },
      s_a, thread::priority(1)
   );

   thread spinner(
      [&]{
         auto const spin_until = time::now() + time::from_milliseconds(200);
         while (time::now() < spin_until) {
            if (woke.load(std::memory_order_acquire)) {
               spinner_saw_wake.store(true, std::memory_order_relaxed);
               break;
            }
         }
      },
      s_b, thread::priority(5)
   );

   kernel::start();

   EXPECT_TRUE(woke.load());
   EXPECT_TRUE(spinner_saw_wake.load()); // observed mid-spin, so the wake preempted
}
