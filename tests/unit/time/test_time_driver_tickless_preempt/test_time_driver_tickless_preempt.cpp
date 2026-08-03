/**
 * @file test_time_driver_tickless_preempt.cpp
 * @brief Tickless driver over the REAL linux_preempt timer.
 *
 * Complement to the coop-backed, hand-pumped tickless test. Here the tickless
 * driver runs on linux_preempt, arming a genuine POSIX one-shot per deadline and
 * re-arming it from the ISR. This exercises the real arm / disarm / re-arm cycle
 * and signal delivery that the deterministic pump cannot.
 *
 * Timing tests: loose one-sided bounds, not exact counts. now() is
 * CLOCK_MONOTONIC and cannot be pumped. Single core, so time::start() on the
 * test thread targets this thread's own one-shot.
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

class TicklessPreemptTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // Frequency matches the port clock (1 MHz), so from_milliseconds() converts
      // to the same ticks the one-shot is armed against.
      cyros::time::initialise(1'000'000 /* Hz */);
      cyros::time::start();
   }

   void TearDown() override
   {
      cyros::time::stop();
      cyros::time::finalise();
   }
};

// A one-shot arms a real timer and fires exactly once.
TEST_F(TicklessPreemptTest, OneShotFiresOnce)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(10),
                                     counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   std::this_thread::sleep_for(60ms);
   EXPECT_EQ(count.load(), 1);
}

// A recurring timer re-arms its one-shot from the ISR and fires repeatedly.
TEST_F(TicklessPreemptTest, RecurringReArmsAndFiresRepeatedly)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_recurring(cyros::time::from_milliseconds(5),
                                            counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   std::this_thread::sleep_for(120ms);
   cyros::time::cancel(h);

   int const fired = count.load();
   EXPECT_GE(fired, 3);
   EXPECT_LE(fired, 200);
}

// The earliest of several deadlines is the one that fires first: arming keeps the
// minimum. Two one-shots, the nearer fires before the farther.
TEST_F(TicklessPreemptTest, EarliestDeadlineFiresFirst)
{
   std::atomic<int> near_count{0};
   std::atomic<int> far_count{0};

   auto far_h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(60),
                                         counting_callback, &far_count);
   auto near_h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(10),
                                          counting_callback, &near_count);
   ASSERT_NE(far_h.id, 0u);
   ASSERT_NE(near_h.id, 0u);

   std::this_thread::sleep_for(30ms);
   EXPECT_EQ(near_count.load(), 1);   // near fired
   EXPECT_EQ(far_count.load(), 0);    // far not yet

   std::this_thread::sleep_for(50ms);
   EXPECT_EQ(far_count.load(), 1);    // far fired later
}

// Cancelling the only armed one-shot disarms the timer: nothing fires.
TEST_F(TicklessPreemptTest, CancelDisarmsAndPreventsFire)
{
   std::atomic<int> count{0};
   auto h = cyros::time::schedule_at(cyros::time::now() + cyros::time::from_milliseconds(40),
                                     counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   ASSERT_TRUE(cyros::time::cancel(h));
   std::this_thread::sleep_for(70ms);
   EXPECT_EQ(count.load(), 0);
}

}  // namespace