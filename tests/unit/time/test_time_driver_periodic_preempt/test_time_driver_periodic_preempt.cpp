/**
 * @file test_time_driver_periodic_preempt.cpp
 * @brief Periodic driver over the REAL linux_preempt timer.
 *
 * The coop-backed periodic test pumps the ISR by hand and is fully
 * deterministic. This suite is the complement: it runs the periodic driver on
 * linux_preempt, where the timer is a genuine POSIX timer delivering a signal
 * asynchronously. It exercises the parts the pumped test cannot: timer_create,
 * per-core arming, real signal delivery, and the registered-handler routing.
 *
 * These are timing tests, so they assert loose, one-sided bounds (a callback
 * fired at least this many times, exactly once, or never) rather than exact
 * counts. now() here is CLOCK_MONOTONIC, so time advances on its own and cannot
 * be pumped.
 *
 * Single core: get_core_id() is 0 before any kernel bring-up, so time::start()
 * on the test thread creates and targets this thread's own timer.
 */

#include <cyros/time/time.hpp>
#include <cyros/port/port_time.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

using namespace cyros;
using namespace std::chrono_literals;

namespace
{

void counting_callback(void* arg) noexcept
{
   static_cast<std::atomic<int>*>(arg)->fetch_add(1, std::memory_order_relaxed);
}

class PeriodicPreemptTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // 1 kHz tick: the periodic interrupt fires every millisecond, which bounds
      // the granularity at which scheduled deadlines are observed.
      cyros::time::initialise(1'000 /* Hz */);
      cyros::time::start();
   }

   void TearDown() override
   {
      cyros::time::stop();
      cyros::time::finalise();
   }
};

// A one-shot fires exactly once under the real timer and never again.
TEST_F(PeriodicPreemptTest, OneShotFiresOnce)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(10),
                                     counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   std::this_thread::sleep_for(60ms);
   EXPECT_EQ(count.load(), 1);
}

// A recurring callback fires repeatedly, proving the slot re-arms itself rather
// than firing once like a one-shot.
TEST_F(PeriodicPreemptTest, RecurringFiresRepeatedly)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_recurring(cyros::time::from_milliseconds(5),
                                            counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   std::this_thread::sleep_for(120ms);   // ~24 fires at 5 ms; assert well below
   cyros::time::cancel(h);

   int const fired = count.load();
   EXPECT_GE(fired, 3);    // proves recurrence, generous lower bound for loaded CI
   EXPECT_LE(fired, 200);  // sanity ceiling
}

// Cancelling a recurring timer stops further fires.
TEST_F(PeriodicPreemptTest, CancelStopsRecurring)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_recurring(cyros::time::from_milliseconds(5),
                                            counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   std::this_thread::sleep_for(40ms);
   ASSERT_TRUE(cyros::time::cancel(h));

   int const after_cancel = count.load();
   std::this_thread::sleep_for(40ms);
   // No further fires after cancel (allow the one possibly in flight at cancel).
   EXPECT_LE(count.load(), after_cancel + 1);
}

// A one-shot cancelled before its deadline never fires.
TEST_F(PeriodicPreemptTest, CancelBeforeDeadlinePreventsFire)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(50),
                                     counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   ASSERT_TRUE(cyros::time::cancel(h));
   std::this_thread::sleep_for(80ms);
   EXPECT_EQ(count.load(), 0);
}

// now() advances on its own from the real monotonic clock.
TEST_F(PeriodicPreemptTest, NowAdvancesMonotonically)
{
   uint64_t const t0 = cyros::time::now().value;
   std::this_thread::sleep_for(20ms);
   uint64_t const t1 = cyros::time::now().value;
   EXPECT_GT(t1, t0);
}

}  // namespace
