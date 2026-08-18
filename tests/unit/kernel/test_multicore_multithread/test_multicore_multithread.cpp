// test_multicore.cpp
#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (atleast) quad core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);

   int result = RUN_ALL_TESTS();

   return result;
}


class MultiCoreMultiThread_Test : public ::testing::Test
{
   void SetUp() override
   {
      kernel::initialise();
   }

   void TearDown() override
   {
      kernel::finalise();
   }
};



TEST_F(MultiCoreMultiThread_Test,
       GivenTwoCoresAndOneThreadPinnedToEach_WhenKernelStarts_ThenBothThreadsRunOnExpectedCore)
{
   cyros::test::guarded_stack s0;
   cyros::test::guarded_stack s1;

   bool ran0 = false;
   bool ran1 = false;
   uint32_t seen_core0 = std::numeric_limits<uint32_t>::max();
   uint32_t seen_core1 = std::numeric_limits<uint32_t>::max();

   // GIVEN:

   thread t0(
      [&]{
         seen_core0 = this_core::id();
         ran0 = true;
      },
      s0,
      thread::priority(0),
      core0
   );

   thread t1(
      [&]{
         seen_core1 = this_core::id();
         ran1 = true;
      },
      s1,
      thread::priority(0),
      core1
   );

   EXPECT_EQ(kernel::active_threads(), 2u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_EQ(kernel::active_threads(), 0u);
   EXPECT_TRUE(ran0);
   EXPECT_TRUE(ran1);

   EXPECT_EQ(seen_core0, 0u);
   EXPECT_EQ(seen_core1, 1u);
}


TEST_F(MultiCoreMultiThread_Test,
       GivenTwoCores_WhenCore0CreatesAThreadPinnedToCore1AfterStart_ThenCore1RunsIt)
{
   cyros::test::guarded_stack s_creator;
   cyros::test::guarded_stack s_remote;

   bool remote_ran = false;
   uint32_t remote_seen_core = std::numeric_limits<uint32_t>::max();

   thread remote_thread;

   // GIVEN:

   thread creator(
      [&]{
         EXPECT_EQ(this_core::id(), 0u);

         remote_thread = thread(
            [&]{
               remote_seen_core = this_core::id();
               remote_ran = true;
            },
            s_remote,
            thread::priority(0),
            core1
         );
         // We can happily terminate here as remote_thread will be started on core1
      },
      s_creator,
      thread::priority(0),
      core0
   );

   // Only one thread should be registered (creator) as remote_thread handle is empty
   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(remote_ran)
      << "Remote thread never ran (missing intake poke / IPI / idle wake)";
   EXPECT_EQ(remote_seen_core, 1u);
}


TEST_F(MultiCoreMultiThread_Test,
       GivenUpToFourCoresWithOneThreadEach_WhenKernelStarts_ThenAllCoresMakeProgress)
{
   if (kernel::core_count() < 4)  GTEST_SKIP() << "Need at least 4 cores for this test";

   cyros::test::guarded_stack s0;
   cyros::test::guarded_stack s1;
   cyros::test::guarded_stack s2;
   cyros::test::guarded_stack s3;

   std::array<int, 4> stages{0};

   auto make_thread = [&](uint32_t core_id, cyros::test::guarded_stack& stack) -> thread
   {
      return {
         [&, core_id]{
            // stage 1: started
            stages[core_id] = 1;

            // Do some cooperative stepping so reschedule/rotation is exercised.
            for (int i = 0; i < 3; ++i) {
               this_thread::yield();
            }

            // stage 2: finished
            stages[core_id] = 2;
         },
         stack,
         thread::priority(0),
         core_affinity::from_id(core_id)
      };
   };

   // GIVEN:

   auto t1 = make_thread(0, s0);
   auto t2 = make_thread(1, s1);
   auto t3 = make_thread(2, s2);
   auto t4 = make_thread(3, s3);

   // WHEN:

   kernel::start();

   // THEN:

   for (std::uint32_t core_id = 0; core_id < kernel::core_count(); ++core_id) {
      EXPECT_EQ(stages[core_id], 2)
         << "Core " << core_id << " did not complete its thread";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}

TEST_F(MultiCoreMultiThread_Test,
       GivenTwoCores_WhenCore0PokesCore1WhileCore1IsIdle_ThenCore1WakesAndRunsQueuedWork)
{
   cyros::test::guarded_stack s_core0;
   cyros::test::guarded_stack s_core1_work;

   bool core1_work_ran = false;
   thread core1_work; // Empty handle

   // GIVEN:

   // Make core1 have *no* initial threads queued pre-start by creating the core1 work post-start.
   // core1 will start in idle unless/until it receives intake work + IPI.
   thread core0_thread(
      [&]{
         EXPECT_EQ(this_core::id(), 0u);

         // Give core1 a chance to enter idle first (cooperative).
         for (int i = 0; i < 3; ++i) this_thread::yield();

         core1_work = thread(
            [&]{
               core1_work_ran = true;
            },
            s_core1_work,
            thread::priority(0),
            core1
         );

         // Yield to allow IPI -> idle wake -> intake drain -> thread run.
         for (int i = 0; i < 10; ++i) this_thread::yield();
      },
      s_core0,
      thread::priority(0),
      core0
   );

   EXPECT_EQ(kernel::active_threads(), 1u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(core1_work_ran)
      << "core1 did not wake from idle to run queued work (possible missing condvar poke)";
}

/*** Explicit termination via this_thread::thread_exit(), across cores ***/

TEST_F(MultiCoreMultiThread_Test,
       GivenJoinersOnThreeCores_WhenTargetOnCore0CallsThreadExit_ThenEveryJoinerIsWokenAndReturns)
{
   cyros::test::guarded_stack s_target;
   std::array<cyros::test::guarded_stack, 3> s_joiners;

   std::atomic<bool> target_exited{false};
   std::array<std::atomic<bool>, 3> joiner_returned{};

   // GIVEN:

   // The retire runs on core0's arbiter, so each joiner is readied by a
   // cross-core intake post and an IPI rather than a local enqueue.
   thread target(
      [&]{
         for (int i = 0; i < 20; ++i) this_thread::yield(); // let the joiners park
         target_exited.store(true, std::memory_order_release);
         this_thread::thread_exit();
      },
      s_target,
      thread::priority(1),
      core0
   );

   std::vector<thread> joiners;
   joiners.reserve(s_joiners.size());
   auto const affinities = std::to_array({core1, core2, core3});

   for (std::size_t i = 0; i < s_joiners.size(); ++i) {
      joiners.emplace_back(
         [&, i]{
            target.join();
            ASSERT_TRUE(target_exited.load(std::memory_order_acquire));
            joiner_returned[i].store(true, std::memory_order_release);
         },
         s_joiners[i],
         thread::priority(0),
         affinities[i]
      );
   }

   ASSERT_EQ(kernel::active_threads(), 4u);

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_TRUE(target_exited.load(std::memory_order_acquire));
   for (std::size_t i = 0; i < joiner_returned.size(); ++i) {
      EXPECT_TRUE(joiner_returned[i].load(std::memory_order_acquire))
         << "joiner " << i << " was never woken by the remote thread_exit";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}

TEST_F(MultiCoreMultiThread_Test,
       GivenEveryCoreRunningThreadsThatExitExplicitly_WhenSystemRuns_ThenAllTerminateAndTheRunWindsDown)
{
   std::array<cyros::test::guarded_stack, 4> stacks;

   std::array<std::atomic<int>, 4> stages{};

   // GIVEN:

   // The LAST of these to retire is what drives the port's quiesce detection,
   // and under this design that decision is made inside the arbiter, after the
   // retiring thread's transition has already landed.
   std::vector<thread> threads;
   threads.reserve(stacks.size());
   auto const affinities = std::to_array({core0, core1, core2, core3});

   for (std::size_t i = 0; i < stacks.size(); ++i) {
      threads.emplace_back(
         [&, i]{
            stages[i].store(1, std::memory_order_release);
            this_thread::yield();
            stages[i].store(2, std::memory_order_release);
            this_thread::thread_exit();
         },
         stacks[i],
         thread::priority(0),
         affinities[i]
      );
   }

   // WHEN:

   kernel::start();

   // THEN:

   for (std::size_t i = 0; i < stages.size(); ++i) {
      EXPECT_EQ(stages[i].load(std::memory_order_acquire), 2)
         << "core " << i << " did not complete its thread";
   }
   EXPECT_EQ(kernel::active_threads(), 0u);
}
