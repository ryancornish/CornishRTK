/**
 * @file test_diagnostics_deadlock.cpp
 * @brief The wait-for cycle case, in its own binary because it cannot quiesce.
 *
 * A real deadlock is built (two threads, two mutexes, opposite order), the
 * observer confirms the diagnosis, and then the PROCESS exits: the deadlocked
 * threads can never terminate, so kernel::start() would never return and a
 * normal gtest run would hang. The exit code carries the verdict, 0 when every
 * expectation held, 1 otherwise, which is exactly what the test runner grades.
 * One test per binary, nothing can run after the exit.
 */

#include <cyros/kernel/diagnostics.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdlib>

using namespace cyros;

namespace
{
alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;
alignas(16) std::array<std::byte, 64 * 1024> s_o;
} // namespace

TEST(DiagnosticsDeadlock_Test, GivenATwoThreadCycle_WhenQueried_ThenBothChainsConfirmDeadlock)
{
   struct
   {
      sync::mutex m1;
      sync::mutex m2;
      sync::semaphore gate_b{0};
      thread* pa{nullptr};
      thread* pb{nullptr};
   } t;

   kernel::initialise();

   thread b(
      [&t]{
         t.gate_b.acquire();
         t.m2.lock();
         t.m1.lock(); // Blocks behind A, forever
         ADD_FAILURE() << "the deadlocked thread ran on";
      },
      s_b, thread::priority(2)
   );

   thread a(
      [&t]{
         t.m1.lock();
         t.gate_b.release(); // B preempts, takes m2, blocks on m1
         t.m2.lock();        // Blocks behind B: the cycle closes here, forever
         ADD_FAILURE() << "the deadlocked thread ran on";
      },
      s_a, thread::priority(3)
   );

   thread observer(
      [&t]{
         // Runs only once A and B are both parked: it is the least urgent.
         auto const of_a = diagnostics::blocking_chain_of(*t.pa);
         EXPECT_TRUE(of_a.deadlocked);
         ASSERT_EQ(of_a.length, 1u);
         EXPECT_EQ(of_a.holders[0], t.pb->get_id());

         auto const of_b = diagnostics::blocking_chain_of(*t.pb);
         EXPECT_TRUE(of_b.deadlocked);
         ASSERT_EQ(of_b.length, 1u);
         EXPECT_EQ(of_b.holders[0], t.pa->get_id());

         // A confirmed deadlock is permanent, so asking again answers the same.
         EXPECT_TRUE(diagnostics::blocking_chain_of(*t.pa).deadlocked);

         std::exit(::testing::Test::HasFailure() ? 1 : 0);
      },
      s_o, thread::priority(10)
   );

   t.pa = &a;
   t.pb = &b;

   kernel::start(); // Never returns: two threads never terminate
}
