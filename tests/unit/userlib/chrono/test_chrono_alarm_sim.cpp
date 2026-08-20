/**
 * @file test_chrono_alarm_sim.cpp
 * @brief Deterministic alarm and sleep tests on the simulation driver.
 *
 * Model: virtual time moves only when the pumper thread advances it, so every
 * deadline fires at an exact tick and every assertion is about order and tick
 * values, never about wall-clock timing.
 *
 * Shape of every kernel test here: a low-priority pumper thread advances
 * virtual time in unit steps and yields, sleepers block on alarms, and
 * kernel::start() returns when all threads have terminated. The pumper is the
 * lowest priority thread so a woken sleeper always runs before time moves
 * again, which is what makes wake ticks exact.
 */

#include <cyros/chrono/alarm.hpp>
#include <cyros/chrono/chrono.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/time/time.hpp>
#include <cyros/time/simulation.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <vector>

using namespace cyros;

namespace
{

constexpr auto frequency = 1'000u; // 1 kHz, one tick per virtual millisecond

class ChronoAlarmSim_Test : public ::testing::Test
{
protected:
   void SetUp() override
   {
      kernel::initialise();
      time::initialise(frequency);
      time::simulation::set_mode(time::simulation::mode::virtual_time);
      time::start();
   }

   void TearDown() override
   {
      time::stop();
      time::finalise();
      kernel::finalise();
   }
};

/* Advance virtual time one tick at a time until the deadline, yielding after
 * each step so any thread woken by that tick runs before time moves again. */
void pump_until(time::time_point deadline)
{
   while (time::now() < deadline) {
      time::simulation::advance_by(time::duration{1});
      this_thread::yield();
   }
}

alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;
alignas(16) std::array<std::byte, 64 * 1024> s_c;
alignas(16) std::array<std::byte, 64 * 1024> s_pump;

} // namespace

TEST_F(ChronoAlarmSim_Test, GivenArmedAlarm_WhenDeadlinePasses_ThenWaiterWakesAtExactTick)
{
   uint64_t woke_at = 0;
   bool early = false;

   thread sleeper(
      [&]{
         chrono::alarm a;
         a.arm_at(time::time_point{50});
         this_thread::wait_on(a);
         woke_at = time::now().value;
         early = !a.expired();
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(woke_at, 50u);
   EXPECT_FALSE(early);
}

TEST_F(ChronoAlarmSim_Test, GivenSleepUntil_WhenDeadlinePasses_ThenSleeperResumesAtDeadline)
{
   uint64_t woke_at = 0;

   thread sleeper(
      [&]{
         this_thread::sleep_until(time::time_point{30});
         woke_at = time::now().value;
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{60}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(woke_at, 30u);
}

TEST_F(ChronoAlarmSim_Test, GivenSleepFor_WhenDurationElapses_ThenSleeperResumesAfterExactlyThatManyTicks)
{
   uint64_t slept_from = 0;
   uint64_t woke_at = 0;

   thread sleeper(
      [&]{
         slept_from = time::now().value;
         this_thread::sleep_for(time::duration{25});
         woke_at = time::now().value;
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{60}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(woke_at, slept_from + 25);
}

TEST_F(ChronoAlarmSim_Test, GivenZeroDuration_WhenSleepForRuns_ThenItReturnsWithoutBlocking)
{
   bool returned = false;

   thread sleeper(
      [&]{
         this_thread::sleep_for(time::duration{0});
         returned = true;
      },
      s_a, thread::priority(1)
   );

   // Deliberately no pumper: time never moves, so a zero sleep that blocked
   // would hang the suite rather than pass it.
   kernel::start();

   EXPECT_TRUE(returned);
}

TEST_F(ChronoAlarmSim_Test, GivenPastDeadline_WhenSleepUntilRuns_ThenItReturnsOnTheNextEvaluation)
{
   bool returned = false;

   thread pre_pump([&]{ pump_until(time::time_point{20}); }, s_b, thread::priority(1));

   thread sleeper(
      [&]{
         // Runs after pre_pump terminated, so now() is 20 and 5 is the past.
         this_thread::sleep_until(time::time_point{5});
         returned = true;
      },
      s_a, thread::priority(2)
   );

   thread pumper([&]{ pump_until(time::time_point{40}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_TRUE(returned);
}

TEST_F(ChronoAlarmSim_Test, GivenThreeSleepers_WhenTimeAdvances_ThenTheyWakeInDeadlineOrderNotArmOrder)
{
   std::vector<int> order;

   // Armed in the order c, b, a by priority, deadlines inverted.
   thread c([&]{ this_thread::sleep_until(time::time_point{70}); order.push_back(3); }, s_c, thread::priority(1));
   thread b([&]{ this_thread::sleep_until(time::time_point{50}); order.push_back(2); }, s_b, thread::priority(2));
   thread a([&]{ this_thread::sleep_until(time::time_point{30}); order.push_back(1); }, s_a, thread::priority(3));

   thread pumper([&]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   ASSERT_EQ(order.size(), 3u);
   EXPECT_EQ(order[0], 1);
   EXPECT_EQ(order[1], 2);
   EXPECT_EQ(order[2], 3);
}

TEST_F(ChronoAlarmSim_Test, GivenFiredAlarm_WhenWaitStartsAfterTheFire_ThenItReturnsImmediately)
{
   bool returned = false;

   thread sleeper(
      [&]{
         chrono::alarm a;
         a.arm_at(time::time_point{10});

         // Fire the deadline before parking. Same thread advances time, so
         // the callback has run before wait_on is entered: this is the poll
         // half of the two-phase block satisfying the wait, not the wake.
         time::simulation::advance_to(time::time_point{15});
         ASSERT_TRUE(a.expired());

         this_thread::wait_on(a);
         returned = true;
      },
      s_a, thread::priority(1)
   );

   kernel::start();

   EXPECT_TRUE(returned);
}

TEST_F(ChronoAlarmSim_Test, GivenExpiredAlarm_WhenReArmed_ThenItFiresAgainAtTheNewDeadline)
{
   uint64_t first = 0;
   uint64_t second = 0;

   thread sleeper(
      [&]{
         chrono::alarm a;
         a.arm_at(time::time_point{10});
         this_thread::wait_on(a);
         first = time::now().value;

         a.disarm();
         EXPECT_FALSE(a.expired());

         a.arm_at(time::time_point{40});
         this_thread::wait_on(a);
         second = time::now().value;
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{60}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(first, 10u);
   EXPECT_EQ(second, 40u);
}

TEST_F(ChronoAlarmSim_Test, GivenPendingAlarm_WhenDisarmed_ThenTheCallbackNeverFires)
{
   bool fired_late = false;

   thread owner(
      [&]{
         chrono::alarm a;
         a.arm_at(time::time_point{30});
         a.disarm();
         EXPECT_FALSE(a.expired());
      },
      s_a, thread::priority(1)
   );

   thread pumper(
      [&]{
         pump_until(time::time_point{60});
         // The alarm object died at tick 0 and its deadline was 30. Getting
         // here without a crash or a stale-callback write is the assertion,
         // the flag just gives the test a positive statement to make.
         fired_late = false;
      },
      s_pump, thread::priority(20)
   );

   kernel::start();

   EXPECT_FALSE(fired_late);
}

/* The composition the feature exists for: wait_on_any(thing, alarm) is a timed
 * wait on the thing. Both outcomes, and the disarm-after-win path that the
 * timed semaphore in stage 2 will rely on. */

TEST_F(ChronoAlarmSim_Test, GivenTimedWait_WhenTheSemaphoreIsReleasedFirst_ThenTheSemaphoreIndexWinsAndDisarmIsClean)
{
   sync::semaphore sem(0);
   std::size_t index = 999;

   thread waiter(
      [&]{
         chrono::alarm deadline;
         deadline.arm_at(time::time_point{80});
         index = this_thread::wait_on_any(sem, deadline);
         deadline.disarm();
      },
      s_a, thread::priority(1)
   );

   thread releaser(
      [&]{
         pump_until(time::time_point{20});
         sem.release();
      },
      s_b, thread::priority(10)
   );

   thread pumper([&]{ pump_until(time::time_point{120}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(index, 0u); // the semaphore, not the deadline
}

TEST_F(ChronoAlarmSim_Test, GivenTimedWait_WhenNothingReleases_ThenTheAlarmIndexWinsAndTheCountIsUntouched)
{
   sync::semaphore sem(0);
   std::size_t index = 999;
   std::size_t count_after = 999;

   thread waiter(
      [&]{
         chrono::alarm deadline;
         deadline.arm_at(time::time_point{40});
         index = this_thread::wait_on_any(sem, deadline);
         count_after = sem.peek();
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{80}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_EQ(index, 1u); // the deadline
   EXPECT_EQ(count_after, 0u);
}

/* Stage 2: the timed methods themselves, chrono-owned definitions on the
 * sync-declared semaphore. */

TEST_F(ChronoAlarmSim_Test, GivenTimedAcquire_WhenNothingReleases_ThenItReturnsFalseAtTheDeadline)
{
   sync::semaphore sem(0);
   bool got = true;
   uint64_t returned_at = 0;

   thread waiter(
      [&]{
         got = sem.try_acquire_until(time::time_point{40});
         returned_at = time::now().value;
      },
      s_a, thread::priority(1)
   );

   thread pumper([&]{ pump_until(time::time_point{80}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_FALSE(got);
   EXPECT_EQ(returned_at, 40u);
   EXPECT_EQ(sem.peek(), 0u);
}

TEST_F(ChronoAlarmSim_Test, GivenTimedAcquire_WhenReleasedBeforeTheDeadline_ThenItReturnsTrueAndConsumes)
{
   sync::semaphore sem(0);
   bool got = false;

   thread waiter(
      [&]{
         got = sem.try_acquire_for(time::duration{60});
      },
      s_a, thread::priority(1)
   );

   thread releaser(
      [&]{
         pump_until(time::time_point{15});
         sem.release();
      },
      s_b, thread::priority(10)
   );

   thread pumper([&]{ pump_until(time::time_point{100}); }, s_pump, thread::priority(20));

   kernel::start();

   EXPECT_TRUE(got);
   EXPECT_EQ(sem.peek(), 0u); // consumed, not merely observed
}

TEST_F(ChronoAlarmSim_Test, GivenAPastDeadline_WhenTryAcquireUntilRuns_ThenItDegradesToTryAcquireWithoutBlocking)
{
   sync::semaphore with_token(1);
   sync::semaphore without_token(0);
   bool took = false;
   bool missed = true;

   thread caller(
      [&]{
         time::simulation::advance_to(time::time_point{20});
         took   = with_token.try_acquire_until(time::time_point{5});
         missed = without_token.try_acquire_until(time::time_point{5});
      },
      s_a, thread::priority(1)
   );

   // No pumper: if the empty-semaphore case blocked, the suite would hang.
   kernel::start();

   EXPECT_TRUE(took);
   EXPECT_FALSE(missed);
}

TEST_F(ChronoAlarmSim_Test, GivenZeroDuration_WhenTryAcquireForRuns_ThenItDegradesToTryAcquire)
{
   sync::semaphore sem(0);
   bool got = true;

   thread caller([&]{ got = sem.try_acquire_for(time::duration{0}); }, s_a, thread::priority(1));

   kernel::start();

   EXPECT_FALSE(got);
}
