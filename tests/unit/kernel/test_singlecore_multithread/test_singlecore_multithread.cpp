#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);

   int result = RUN_ALL_TESTS();

   return result;
}

struct ThreadSafeLog
{
   void push(thread::id id)
   {
      std::lock_guard<std::mutex> g(m);
      v.push_back(id);
   }
   std::vector<thread::id> v;
   std::mutex m;
};


TEST(SingleCoreMultiThread_Test,
     GivenTwoEqualPriorityThreads_WhenSystemStarts_ThenThreadsExecuteInRegistrationOrder)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack2{};

   std::vector<thread::id> order;

   kernel::initialise();

   thread t1([&]{ order.push_back(this_thread::id()); }, stack1, thread::priority(0), core0);
   thread t2([&]{ order.push_back(this_thread::id()); }, stack2, thread::priority(0), core0);

   ASSERT_EQ(kernel::active_threads(), 2u) << "Not all threads registered";

   kernel::start();

   EXPECT_EQ(kernel::active_threads(), 0u) << "Not all threads terminated";
   ASSERT_EQ(order.size(), 2u)             << "Not all threads started";
   EXPECT_EQ(order[0], thread::id(1));
   EXPECT_EQ(order[1], thread::id(2));

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenTwoEqualPriorityThreads_WhenEachYields_ThenTheyCooperativelyProgressAlternating)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack2{};

   std::vector<thread::id> order;

   auto worker = [&](int stages)
   {
      for (int i = 0; i < stages; ++i) {
         order.push_back(this_thread::id());
         this_thread::yield();
      }
   };

   kernel::initialise();

   thread t1([&]{ worker(3); }, stack1, thread::priority(0), core0);
   thread t2([&]{ worker(3); }, stack2, thread::priority(0), core0);

   ASSERT_EQ(kernel::active_threads(), 2u);

   kernel::start();

   EXPECT_EQ(kernel::active_threads(), 0u);
   ASSERT_EQ(order.size(), 6u);

   const std::array<thread::id, 6> expected{
      thread::id(1), thread::id(2), thread::id(1), thread::id(2), thread::id(1), thread::id(2)
   };
   for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_EQ(order[i], expected[i]) << "Mismatch at index " << i;
   }

   kernel::finalise();
}


TEST(SingleCoreMultiThread_Test,
     GivenTwoDifferentPriorities_WhenSystemStarts_ThenHigherPriorityRunsFirstEvenIfRegisteredSecond)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack_lo{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack_hi{};

   std::vector<thread::id> order;

   kernel::initialise();

   // Lower priority first (numerically larger == lower priority in your code base as described)
   thread low([&]{ order.push_back(this_thread::id()); }, stack_lo, thread::priority(5), core0);
   thread high([&]{ order.push_back(this_thread::id()); }, stack_hi, thread::priority(0), core0);

   ASSERT_EQ(kernel::active_threads(), 2u);

   kernel::start();

   ASSERT_EQ(order.size(), 2u);
   EXPECT_EQ(order[0], thread::id(2)) << "High priority thread should run first";
   EXPECT_EQ(order[1], thread::id(1)) << "Low priority thread should run second";

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenThreeEqualPriorityThreads_WhenTheyYield_ThenTheyRoundRobinInRegistrationOrder)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s2{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s3{};

   std::vector<thread::id> order;

   auto worker = [&](int stages)
   {
      for (int i = 0; i < stages; ++i) {
         order.push_back(this_thread::id());
         this_thread::yield();
      }
   };

   kernel::initialise();

   thread t1([&]{ worker(3); }, s1, thread::priority(0), core0);
   thread t2([&]{ worker(3); }, s2, thread::priority(0), core0);
   thread t3([&]{ worker(3); }, s3, thread::priority(0), core0);

   ASSERT_EQ(kernel::active_threads(), 3u);

   kernel::start();

   ASSERT_EQ(order.size(), 9u);

   const std::array<thread::id, 9> expected{
      thread::id(1), thread::id(2), thread::id(3),
      thread::id(1), thread::id(2), thread::id(3),
      thread::id(1), thread::id(2), thread::id(3),
   };

   for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_EQ(order[i], expected[i]) << "Mismatch at index " << i;
   }

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenTwoThreads_WhenOneNeverYields_ThenOtherDoesNotRunUntilFirstReturns)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s1{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s2{};

   std::vector<int> markers;

   kernel::initialise();

   thread t1(
      [&]{
         markers.push_back(1); // t1 start
         // No yield here; cooperatively hog until it returns.
         markers.push_back(2); // t1 end
      },
      s1,
      thread::priority(0),
      core0
   );

   thread t2(
      [&]{
         markers.push_back(3); // t2 start
      },
      s2,
      thread::priority(0),
      core0
   );

   kernel::start();

   ASSERT_EQ(markers.size(), 3u);
   EXPECT_EQ(markers[0], 1);
   EXPECT_EQ(markers[1], 2);
   EXPECT_EQ(markers[2], 3) << "Second thread should only run after the first returns (cooperative)";

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenThirtyEqualPriorityThreads_WhenSystemStarts_ThenThreadsObeyRoundRobinRules)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::array<std::byte, STACK_SIZE>, 30> stacks{};


   std::vector<thread> threads;
   threads.reserve(stacks.size());
   std::vector<uint32_t> markers;

   kernel::initialise();

   for (auto& stack : stacks) {
      threads.emplace_back(
      [&]{
         markers.push_back(this_thread::id());
         this_thread::yield();
         markers.push_back(this_thread::id());
      },
      stack, thread::priority(0), core0);
   }

   kernel::start();

   ASSERT_EQ(markers.size(), 30u * 2);

   auto expected_order = std::to_array<uint32_t>({
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
   });

   for (unsigned i = 0; i < markers.size(); ++i) {
      EXPECT_EQ(markers[i], expected_order[i]);
   }

   kernel::finalise();
}


TEST(SingleCoreMultiThread_Test,
     GivenThirtyDifferentPriorityThreads_WhenSystemStarts_ThenThreadsExecuteInPriorityOrder)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::array<std::byte, STACK_SIZE>, 30> stacks{};


   std::vector<thread> threads;
   threads.reserve(stacks.size());
   std::vector<uint32_t> markers;

   kernel::initialise();

   for (unsigned prio = 29; auto& stack : stacks) {
      threads.emplace_back(
      [&]{
         markers.push_back(this_thread::id());
         this_thread::yield(); // Should reenqueue same thread leading to double number pushback
         markers.push_back(this_thread::id());
      },
      stack, thread::priority(prio--), core0);
   }

   kernel::start();

   ASSERT_EQ(markers.size(), 30u * 2);

   auto expected_order = std::to_array<uint32_t>({
      30, 30, 29, 29, 28, 28, 27, 27, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 21, 21, 20, 20, 19, 19, 18, 18, 17, 17, 16, 16,
      15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1,
   });

   for (unsigned i = 0; i < markers.size(); ++i) {
      EXPECT_EQ(markers[i], expected_order[i]);
   }

   kernel::finalise();
}


TEST(SingleCoreMultiThread_Test,
     GivenSingleThread_WhenThreadCreatesAnotherThreadOfHigherPriority_ThenThreadIsImmediatelyPreempted)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_creator{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_child{};

   thread child_thread;

   std::vector<int> marker;

   // GIVEN:

   kernel::initialise();

   thread creator(
      [&]{
         marker.push_back(10); // 10 is the priority of the creator thread and marks when it ran

         child_thread = thread(
            [&]{
               marker.push_back(9);
            },
            s_child,
            thread::priority(9),
            core0
         );

         marker.push_back(10);
      },
      s_creator,
      thread::priority(10),
      core0
   );

   // Only one thread should be registered (creator) as child_thread handle is empty
   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(marker.size(), 3u);
   EXPECT_EQ(marker[0], 10u);
   EXPECT_EQ(marker[1], 9u)
      << "creator_thread was not preempted by just-created child_thread";
   EXPECT_EQ(marker[2], 10u);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenSingleThread_WhenThreadCreatesAnotherThreadOfLowerPriority_ThenCreatedThreadDoesNotRunUntilFirstThreadFinishes)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_creator{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_child{};

   thread child_thread;

   std::vector<int> marker;

   // GIVEN:

   kernel::initialise();

   thread creator(
      [&]{
         marker.push_back(10); // 10 is the priority of the creator thread and marks when it ran

         child_thread = thread(
            [&]{
               marker.push_back(11);
            },
            s_child,
            thread::priority(11),
            core0
         );

         marker.push_back(10);
      },
      s_creator,
      thread::priority(10),
      core0
   );

   // Only one thread should be registered (creator) as child_thread handle is empty
   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(marker.size(), 3u);
   EXPECT_EQ(marker[0], 10u);
   EXPECT_EQ(marker[1], 10u)
      << "creator_thread was wrongfully preempted by just-created child_thread";
   EXPECT_EQ(marker[2], 11u);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenSingleThread_WhenThreadCreatesAnotherThreadOfEqualPriority_ThenCreatedThreadDoesNotRunUntilFirstThreadYields)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_creator{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_child{};

   thread child_thread;

   std::vector<uint32_t> marker;

   // GIVEN:

   kernel::initialise();

   thread creator(
      [&]{
         marker.push_back(this_thread::id());

         child_thread = thread(
            [&]{
               marker.push_back(this_thread::id());
               this_thread::yield();
               marker.push_back(this_thread::id());
            },
            s_child,
            thread::priority(10),
            core0
         );

         marker.push_back(this_thread::id());
         this_thread::yield();
         marker.push_back(this_thread::id());
      },
      s_creator,
      thread::priority(10),
      core0
   );

   // Only one thread should be registered (creator) as child_thread handle is empty
   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(marker.size(), 5u);
   EXPECT_EQ(marker[0], 1u);
   EXPECT_EQ(marker[1], 1u);
   EXPECT_EQ(marker[2], 2u);
   EXPECT_EQ(marker[3], 1u);
   EXPECT_EQ(marker[4], 2u);

   kernel::finalise();
}



/*** Thread joining ***/

TEST(SingleCoreMultiThread_Test,
     GivenJoinerHigherPriority_WhenItJoinsTarget_ThenJoinerBlocksUntilTargetTerminates)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> target_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> joiner_stack{};

   std::atomic<bool> target_started{false};
   std::atomic<bool> target_finished{false};
   std::atomic<bool> joiner_returned{false};

   // GIVEN:

   thread target(
      [&]{
         target_started.store(true, std::memory_order_release);
         this_thread::yield(); // SHould immediately resume after reschedule
         target_finished.store(true, std::memory_order_release);
      },
      target_stack,
      thread::priority(3),
      core0
   );

   thread joiner(
      [&]{
         // Join should block until target exits.
         target.join();
         joiner_returned.store(true, std::memory_order_release);

         // After join returns, target must be finished.
         ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
      },
      joiner_stack,
      thread::priority(0), // higher priority
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(target_started.load(std::memory_order_acquire));
   ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
   ASSERT_TRUE(joiner_returned.load(std::memory_order_acquire));
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenTwoJoinersDifferentPriorities_WhenBothJoinSameTarget_ThenBothBlockAndReturnAfterTargetTerminates)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> target_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> joiner_hi_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> joiner_lo_stack{};

   std::atomic<int> phase{0}; // monotonic progress marker
   std::atomic<bool> target_finished{false};
   std::atomic<bool> joiner_hi_returned{false};
   std::atomic<bool> joiner_lo_returned{false};

   // GIVEN:

   thread target(
      [&]{
         phase.store(1, std::memory_order_release);
         this_thread::yield();
         target_finished.store(true, std::memory_order_release);
         phase.store(2, std::memory_order_release);
      },
      target_stack,
      thread::priority(4),
      core0
   );

   thread joiner_hi(
      [&]{
         // This should block immediately, allowing target and other joiner to run.
         phase.store(10, std::memory_order_release);
         target.join();
         joiner_hi_returned.store(true, std::memory_order_release);
         ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
         phase.store(11, std::memory_order_release);
      },
      joiner_hi_stack,
      thread::priority(0),
      core0
   );

   thread joiner_lo(
      [&]{
         phase.store(20, std::memory_order_release);
         target.join();
         joiner_lo_returned.store(true, std::memory_order_release);
         ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
         phase.store(21, std::memory_order_release);
      },
      joiner_lo_stack,
      thread::priority(2),
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
   ASSERT_TRUE(joiner_hi_returned.load(std::memory_order_acquire));
   ASSERT_TRUE(joiner_lo_returned.load(std::memory_order_acquire));
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenJoinerLowerPriority_WhenTargetTerminatesBeforeJoinCall_ThenJoinReturnsImmediately)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> target_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> joiner_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> helper_stack{};

   std::atomic<bool> target_finished{false};
   std::atomic<bool> joiner_called_join{false};
   std::atomic<bool> joiner_returned{false};

   // GIVEN:

   thread target(
      [&]{
         // Finish immediately
         target_finished.store(true, std::memory_order_release);
      },
      target_stack,
      thread::priority(0), // higher priority so it runs and finishes first
      core0
   );

   // Helper to yield enough times so target definitely runs before joiner.
   thread helper(
      [&]{
         // With target prio 0, it will run before us anyway, this just adds reschedule points.
         this_thread::yield();
         this_thread::yield();
      },
      helper_stack,
      thread::priority(1),
      core0
   );

   thread joiner(
      [&]{
         joiner_called_join.store(true, std::memory_order_release);
         target.join(); // should return immediately because target already terminated
         joiner_returned.store(true, std::memory_order_release);
         ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
      },
      joiner_stack,
      thread::priority(3), // lower priority than target/helper
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(target_finished.load(std::memory_order_acquire));
   ASSERT_TRUE(joiner_called_join.load(std::memory_order_acquire));
   ASSERT_TRUE(joiner_returned.load(std::memory_order_acquire));
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenChainJoinAWaitsBWaitsC_WhenSystemStarts_ThenChainUnblocksInOrderAndAllTerminate)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack_a{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack_b{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> stack_c{};

   std::atomic<int> order{0}; // record completion order
   std::atomic<bool> a_done{false};
   std::atomic<bool> b_done{false};
   std::atomic<bool> c_done{false};

   // GIVEN:

   // Construct in C, B, A order so lambdas can reference already-created objects.
   thread c(
      [&]{
         // finish last dependency first
         c_done.store(true, std::memory_order_release);
         order.fetch_add(1, std::memory_order_acq_rel);
      },
      stack_c,
      thread::priority(2),
      core0
   );

   thread b(
      [&]{
         c.join();
         b_done.store(true, std::memory_order_release);
         order.fetch_add(1, std::memory_order_acq_rel);
         ASSERT_TRUE(c_done.load(std::memory_order_acquire));
      },
      stack_b,
      thread::priority(1),
      core0
   );

   thread a(
      [&]{
         b.join();
         a_done.store(true, std::memory_order_release);
         order.fetch_add(1, std::memory_order_acq_rel);
         ASSERT_TRUE(b_done.load(std::memory_order_acquire));
         ASSERT_TRUE(c_done.load(std::memory_order_acquire));
      },
      stack_a,
      thread::priority(0), // highest priority: will block immediately on join, allowing others to run
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_TRUE(a_done.load(std::memory_order_acquire));
   ASSERT_TRUE(b_done.load(std::memory_order_acquire));
   ASSERT_TRUE(c_done.load(std::memory_order_acquire));
   ASSERT_EQ(order.load(std::memory_order_acquire), 3);
   ASSERT_EQ(kernel::active_threads(), 0U);

   kernel::finalise();
}


/*** Explicit termination via this_thread::thread_exit() ***/

/* thread_exit() and falling off the end of an entry function are the same
 * operation: both record thread_disposition::terminating and hand the thread to
 * the reschedule arbiter, which performs the transition. These cover the
 * explicit spelling, which the automatic one exercises on every other test in
 * the suite. */

TEST(SingleCoreMultiThread_Test,
     GivenThreadCallingThreadExit_WhenItExits_ThenTheRestOfItsEntryFunctionDoesNotRun)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_exiter{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_other{};

   std::vector<int> markers;

   // GIVEN:

   thread exiter(
      [&]{
         markers.push_back(1);
         this_thread::thread_exit();
         markers.push_back(99); // Unreachable: the arbiter never picks us again
      },
      s_exiter,
      thread::priority(0),
      core0
   );

   thread other(
      [&]{
         markers.push_back(2);
      },
      s_other,
      thread::priority(1),
      core0
   );

   ASSERT_EQ(kernel::active_threads(), 2u);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(markers.size(), 2u) << "code after thread_exit() ran";
   EXPECT_EQ(markers[0], 1);
   EXPECT_EQ(markers[1], 2) << "the lower priority thread did not get the core after the exit";
   EXPECT_EQ(kernel::active_threads(), 0u);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenThreadExitCalledFromANestedFrame_WhenItExits_ThenTheThreadStillTerminates)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> s_exiter{};

   std::atomic<bool> reached_inner{false};
   std::atomic<bool> returned_from_inner{false};

   // GIVEN:

   // Exiting does NOT unwind: the frames below are abandoned where they stand,
   // exactly as they are on a thread that is preempted and never resumed.
   auto inner = [&]{
      reached_inner.store(true, std::memory_order_release);
      this_thread::thread_exit();
   };

   thread exiter(
      [&]{
         inner();
         returned_from_inner.store(true, std::memory_order_release);
      },
      s_exiter,
      thread::priority(0),
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(reached_inner.load(std::memory_order_acquire));
   EXPECT_FALSE(returned_from_inner.load(std::memory_order_acquire))
      << "thread_exit() returned to its caller";
   EXPECT_EQ(kernel::active_threads(), 0u);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenJoinerBlockedOnTarget_WhenTargetCallsThreadExit_ThenTheJoinerIsWokenAndReturns)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> target_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> joiner_stack{};

   std::atomic<bool> target_reached_exit{false};
   std::atomic<bool> joiner_returned{false};

   // GIVEN:

   thread target(
      [&]{
         this_thread::yield(); // let the joiner park on us first
         target_reached_exit.store(true, std::memory_order_release);
         this_thread::thread_exit();
      },
      target_stack,
      thread::priority(3),
      core0
   );

   thread joiner(
      [&]{
         target.join();
         joiner_returned.store(true, std::memory_order_release);

         // The wake is issued by the arbiter AFTER the state transition, so a
         // returning joiner always observes a finished thread.
         ASSERT_TRUE(target_reached_exit.load(std::memory_order_acquire));
      },
      joiner_stack,
      thread::priority(0), // higher, so it blocks on the join before target runs
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(target_reached_exit.load(std::memory_order_acquire));
   EXPECT_TRUE(joiner_returned.load(std::memory_order_acquire))
      << "join() did not return, so thread_exit() did not signal the termination waitable";
   EXPECT_EQ(kernel::active_threads(), 0u);

   kernel::finalise();
}

TEST(SingleCoreMultiThread_Test,
     GivenSeveralThreadsExitingBothWays_WhenSystemRuns_ThenAllTerminateAndTheCoreDrains)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::array<std::byte, STACK_SIZE>, 6> stacks{};

   std::vector<thread> threads;
   threads.reserve(stacks.size());
   std::vector<uint32_t> markers;

   // GIVEN:

   // Alternate the two spellings so both retire paths run interleaved on one core.
   for (unsigned i = 0; auto& stack : stacks) {
      bool const explicit_exit = (i % 2) == 0;
      threads.emplace_back(
         [&, explicit_exit]{
            markers.push_back(this_thread::id());
            this_thread::yield();
            markers.push_back(this_thread::id());
            if (explicit_exit) this_thread::thread_exit();
         },
         stack, thread::priority(0), core0);
      ++i;
   }

   ASSERT_EQ(kernel::active_threads(), 6u);

   // WHEN:

   kernel::start();

   // THEN:

   ASSERT_EQ(markers.size(), 12u);
   for (unsigned i = 0; i < 6; ++i) {
      EXPECT_EQ(markers[i], i + 1) << "first round not in registration order at " << i;
      EXPECT_EQ(markers[i + 6], i + 1) << "second round not in registration order at " << i;
   }
   EXPECT_EQ(kernel::active_threads(), 0u);

   kernel::finalise();
}
