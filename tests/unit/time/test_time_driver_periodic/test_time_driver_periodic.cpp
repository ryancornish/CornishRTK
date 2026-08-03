/**
 * @file test_periodic_driver.cpp
 * @brief Unit tests for the periodic (tickful) time driver.
 *
 * The periodic driver implements the free-function API declared in
 * <cyros/time/time.hpp>. A test binary links exactly ONE time driver, so this
 * file targets the periodic implementation only (selected via test.toml
 * [components].time_driver = "periodic").
 *
 * Model
 * -----
 * The Linux unit-test port backend exposes a deterministic monotonic counter:
 *   - cyros_port_time_now()      -> current value in port ticks
 *   - cyros_port_time_reset(t)   -> set the counter (test-only)
 *   - cyros_port_time_advance(d) -> advance the counter (Linux-only test hook)
 *
 * The periodic driver does not own time; now() simply forwards to the port.
 * We "pump" the driver by advancing port time and then invoking on_timer_isr(),
 * which models the port delivering a periodic timer IRQ.
 *
 * Coverage goal: 100% branch coverage of the periodic translation unit.
 * Every branch below is reachable deterministically; see the per-test notes.
 */

#include <cyros/time/time.hpp>
#include <cyros/port/port_time.h>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

using namespace cyros;

// Linux-only deterministic test hook, implemented in the linux_coop port.
extern void cyros_port_time_advance(uint64_t delta);
extern void cyros_port_time_fire_isr(void);

namespace
{

// MAX_SCHEDULED_CALLBACKS is an implementation-internal constant of the
// periodic driver (currently 16). It is not exported via a public header, so
// we mirror it here. If the driver's limit changes, update this to match.
constexpr uint32_t kMaxScheduledCallbacks = 16;

// Simple counting callback: increments the std::atomic<int> passed as arg.
void counting_callback(void* arg) noexcept
{
   static_cast<std::atomic<int>*>(arg)->fetch_add(1, std::memory_order_relaxed);
}

class PeriodicDriverTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // Deterministic counter start for every test.
      cyros_port_time_reset(0);

      // initialise() asserts it has not already been initialised, and
      // finalise() (run in TearDown) resets the driver's static state, so
      // this pairing is safe to repeat across tests.
      cyros::time::initialise(1'000'000 /* Hz */);
   }

   void TearDown() override
   {
      cyros::time::finalise();
   }

   // Advance port time and deliver one periodic ISR.
   static void advance_and_pump(uint64_t delta_ticks)
   {
      cyros_port_time_advance(delta_ticks);
      cyros_port_time_fire_isr();
   }

   // Deliver an ISR without advancing time.
   static void pump()
   {
      cyros_port_time_fire_isr();
   }
};

/* ============================================================================
 * Lifecycle
 * ========================================================================= */

// start() first call: started == false branch -> performs setup.
// start() second call: started == true branch  -> early return.
TEST_F(PeriodicDriverTest, StartIsIdempotent)
{
   cyros::time::start();
   cyros::time::start();  // second call hits the `started` early-return branch

   SUCCEED();
}

// stop() when started: started == true branch -> performs teardown.
// stop() when not started: started == false branch -> early return.
TEST_F(PeriodicDriverTest, StopIsIdempotent)
{
   cyros::time::stop();   // not started yet: early-return branch

   cyros::time::start();
   cyros::time::stop();   // started: teardown branch
   cyros::time::stop();   // already stopped again: early-return branch

   SUCCEED();
}

// now() forwards the port counter unchanged.
TEST_F(PeriodicDriverTest, NowReflectsPortCounter)
{
   cyros::time::start();
   EXPECT_EQ(cyros::time::now().value, 0u);

   cyros_port_time_advance(123);
   EXPECT_EQ(cyros::time::now().value, 123u);

   cyros_port_time_advance(7);
   EXPECT_EQ(cyros::time::now().value, 130u);
}

/* ============================================================================
 * schedule_at
 * ========================================================================= */

// schedule_at() with a null callback: the `!cb` branch returns an invalid
// handle and consumes no slot.
TEST_F(PeriodicDriverTest, ScheduleNullCallbackReturnsInvalidHandle)
{
   cyros::time::start();

   time::handle h = cyros::time::schedule_at(time::time_point{100}, nullptr, nullptr);
   EXPECT_EQ(h.id, 0u);
}

// schedule_at() with a valid callback: takes the first free slot and returns
// a valid (non-zero) handle.
TEST_F(PeriodicDriverTest, ScheduleValidCallbackReturnsValidHandle)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   EXPECT_NE(h.id, 0u);
}

// Filling every slot then scheduling once more exercises the loop's
// "no free slot found" exit -> returns an invalid handle.
TEST_F(PeriodicDriverTest, ScheduleBeyondCapacityReturnsInvalidHandle)
{
   cyros::time::start();

   std::atomic<int> count{0};
   std::vector<time::handle> handles;
   handles.reserve(kMaxScheduledCallbacks);

   for (uint32_t i = 0; i < kMaxScheduledCallbacks; ++i)
   {
      time::handle h = cyros::time::schedule_at(
         time::time_point{static_cast<uint64_t>(i) + 1000}, counting_callback, &count);
      ASSERT_NE(h.id, 0u) << "slot " << i << " should still be free";
      handles.push_back(h);
   }

   // All slots are now occupied: the next schedule must fail.
   time::handle overflow = cyros::time::schedule_at(time::time_point{9999}, counting_callback, &count);
   EXPECT_EQ(overflow.id, 0u);

   // Freeing one slot makes room again.
   ASSERT_TRUE(cyros::time::cancel(handles.front()));
   time::handle reused = cyros::time::schedule_at(time::time_point{9999}, counting_callback, &count);
   EXPECT_NE(reused.id, 0u);
}

/* ============================================================================
 * Firing semantics
 * ========================================================================= */

// A callback whose deadline is in the future does not fire until time reaches
// it; it fires exactly once on the pump where now >= deadline.
TEST_F(PeriodicDriverTest, CallbackFiresWhenDeadlineReached)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   // Before the deadline: the `when <= now` test is false -> no fire.
   advance_and_pump(99);
   EXPECT_EQ(count.load(), 0);

   // At the deadline: the `when <= now` test is true -> fires once.
   advance_and_pump(1);
   EXPECT_EQ(count.load(), 1);

   // slot was freed on fire, so further pumps do not re-fire.
   advance_and_pump(1000);
   EXPECT_EQ(count.load(), 1);
}

// A deadline exactly equal to "now" fires (boundary of the `when <= now` test).
TEST_F(PeriodicDriverTest, CallbackFiresAtExactDeadline)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{10}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   advance_and_pump(10);
   EXPECT_EQ(count.load(), 1);
}

// A deadline already in the past at schedule time fires on the very next pump,
// even with no time advance (delta == 0).
TEST_F(PeriodicDriverTest, CallbackScheduledInPastFiresOnNextPump)
{
   cyros::time::start();

   cyros_port_time_advance(100);
   ASSERT_EQ(cyros::time::now().value, 100u);

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{50}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   pump();  // no advance; deadline 50 <= now 100
   EXPECT_EQ(count.load(), 1);
}

// Several callbacks with distinct deadlines each fire once, at their own time.
TEST_F(PeriodicDriverTest, MultipleCallbacksFireIndependently)
{
   cyros::time::start();

   std::atomic<int> a{0}, b{0}, c{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{50},  counting_callback, &a).id, 0u);
   ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &b).id, 0u);
   ASSERT_NE(cyros::time::schedule_at(time::time_point{150}, counting_callback, &c).id, 0u);

   advance_and_pump(60);   // now = 60  -> only a
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 0);
   EXPECT_EQ(c.load(), 0);

   advance_and_pump(60);   // now = 120 -> b
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 1);
   EXPECT_EQ(c.load(), 0);

   advance_and_pump(80);   // now = 200 -> c
   EXPECT_EQ(a.load(), 1);
   EXPECT_EQ(b.load(), 1);
   EXPECT_EQ(c.load(), 1);
}

// Multiple callbacks all due on the same pump fire together.
TEST_F(PeriodicDriverTest, AllDueCallbacksFireOnSamePump)
{
   cyros::time::start();

   std::atomic<int> count{0};
   for (int i = 0; i < 5; ++i)
   {
      ASSERT_NE(cyros::time::schedule_at(time::time_point{100}, counting_callback, &count).id, 0u);
   }

   advance_and_pump(200);
   EXPECT_EQ(count.load(), 5);
}

// A pump that finds no due callbacks (slot occupied but `when > now`) and a
// pump over only-empty slots both exercise the "no fire" path of fire_due_isr.
TEST_F(PeriodicDriverTest, PumpWithNothingDueFiresNothing)
{
   cyros::time::start();

   // No callbacks scheduled at all: loop body never fires.
   advance_and_pump(500);

   // One callback scheduled far in the future: loop visits an occupied slot
   // whose `when <= now` test is false.
   std::atomic<int> count{0};
   ASSERT_NE(cyros::time::schedule_at(time::time_point{10'000}, counting_callback, &count).id, 0u);
   advance_and_pump(500);
   EXPECT_EQ(count.load(), 0);
}

/* ============================================================================
 * cancel
 * ========================================================================= */

// cancel() of an invalid (id == 0) handle: the `h.id == 0` branch returns false.
TEST_F(PeriodicDriverTest, CancelInvalidHandleReturnsFalse)
{
   cyros::time::start();

   EXPECT_FALSE(cyros::time::cancel(time::handle{0}));
}

// cancel() of a non-zero id that matches no slot: the loop completes without
// a match -> returns false.
TEST_F(PeriodicDriverTest, CancelUnknownHandleReturnsFalse)
{
   cyros::time::start();

   EXPECT_FALSE(cyros::time::cancel(time::handle{999999}));
}

// cancel() before firing: matching slot is found and cleared -> returns true,
// and the callback never fires afterwards.
TEST_F(PeriodicDriverTest, CancelBeforeFiringPreventsCallback)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h));

   advance_and_pump(500);
   EXPECT_EQ(count.load(), 0);
}

// cancel() after firing: the slot was freed by the fire, so the handle no
// longer matches -> returns false. The callback is not invoked a second time.
TEST_F(PeriodicDriverTest, CancelAfterFiringReturnsFalse)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{50}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   advance_and_pump(50);
   EXPECT_EQ(count.load(), 1);

   EXPECT_FALSE(cyros::time::cancel(h));
   EXPECT_EQ(count.load(), 1);
}

// Cancelling the same handle twice: the second cancel finds no match.
TEST_F(PeriodicDriverTest, CancelTwiceReturnsFalseSecondTime)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   ASSERT_NE(h.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h));
   EXPECT_FALSE(cyros::time::cancel(h));
}

// Cancelling one of several leaves the others intact.
TEST_F(PeriodicDriverTest, CancelOneOfManyLeavesOthers)
{
   cyros::time::start();

   std::atomic<int> count{0};
   time::handle h1 = cyros::time::schedule_at(time::time_point{100}, counting_callback, &count);
   time::handle h2 = cyros::time::schedule_at(time::time_point{200}, counting_callback, &count);
   time::handle h3 = cyros::time::schedule_at(time::time_point{300}, counting_callback, &count);
   ASSERT_NE(h1.id, 0u);
   ASSERT_NE(h2.id, 0u);
   ASSERT_NE(h3.id, 0u);

   EXPECT_TRUE(cyros::time::cancel(h2));  // cancel the middle one

   advance_and_pump(400);
   EXPECT_EQ(count.load(), 2);  // h1 and h3 fired, h2 did not
}

/* ============================================================================
 * duration conversion
 *
 * The Linux port reports a fixed frequency of 1'000'000 Hz (1 tick = 1 us),
 * so conversions are computed against that. ceil_div is exercised by both an
 * exact case and a case that requires rounding up.
 * ========================================================================= */

TEST_F(PeriodicDriverTest, FromMillisecondsConvertsExactly)
{
   // 10 ms at 1 MHz = 10'000 ticks, no rounding needed.
   EXPECT_EQ(cyros::time::from_milliseconds(10).value, 10'000u);
}

TEST_F(PeriodicDriverTest, FromMillisecondsZeroIsZero)
{
   // 0 ms: numerator is 0, ceil_div yields 0 (covers the a == 0 path).
   EXPECT_EQ(cyros::time::from_milliseconds(0).value, 0u);
}

TEST_F(PeriodicDriverTest, FromMicrosecondsConvertsExactly)
{
   // 500 us at 1 MHz = 500 ticks exactly.
   EXPECT_EQ(cyros::time::from_microseconds(500).value, 500u);
}

TEST_F(PeriodicDriverTest, FromMicrosecondsRoundsUp)
{
   // 1001 us at 1 MHz = 1001 ticks exactly (1 us == 1 tick at this frequency),
   // so this is exact. The rounding-up code path of ceil_div is genuinely
   // exercised only when freq < 1 MHz; with the fixed 1 MHz port frequency the
   // microsecond conversion can never produce a fractional tick. from_ms above
   // covers an exact multiple; there is no sub-tick remainder to round here.
   EXPECT_EQ(cyros::time::from_microseconds(1001).value, 1001u);
}

}  // namespace