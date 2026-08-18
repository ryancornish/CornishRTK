/**
 * @file test_singlecore_preemption.cpp
 * @brief Single-core preemption tests for the linux_preempt port.
 *
 * These exercise the property the cooperative (linux_coop) port structurally cannot
 * reach: an asynchronous signal interrupting a running thread mid-loop and
 * forcing a context switch. Two equal-priority threads are pinned to core0 and
 * neither yields. On a cooperative port the first would run to completion before
 * the second ever started. Here an external interrupt source preempts them, so
 * they interleave.
 *
 * The interrupt source is a plain std::thread that fires the port's reschedule
 * IPI at core0. That stands in for the timer port, which does not exist yet.
 *
 * Lifecycle care
 * --------------
 * A reschedule signal delivered after the kernel has shut down would land on a
 * core whose reschedule handler has been cleared, which is undefined. So the
 * helper must be provably stopped before shutdown can run. The interleaving test
 * below arranges that the helper fires only while both workers are still
 * spinning, and the same flag that stops the helper is the one that releases the
 * workers. Every IPI therefore happens while both workers are alive, which means
 * the kernel cannot have quiesced and the cores are still valid. Once the flag
 * flips, teardown is driven entirely by thread_exit's own self-signal, so there
 * is no liveness dependence on the helper either.
 */

#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (32 * 1024);

namespace
{

cyros::test::guarded_stack a_stack;
cyros::test::guarded_stack b_stack;

}  // namespace


/* ============================================================================
 * No preemption source: two non-yielding threads run sequentially
 *
 * With nothing driving preemption, the first-scheduled thread must run its
 * entire loop before the second runs at all. The first thread watching the
 * second's counter proves it: that counter stays at zero for the whole of the
 * first thread's run. This is the cooperative baseline the preempt port must
 * still honour when no interrupt arrives.
 * ========================================================================= */
TEST(SingleCorePreemption_Test,
     GivenTwoNonYieldingThreadsAndNoPreemptionSource_ThenTheyRunSequentially)
{
   kernel::initialise();

   constexpr std::uint64_t target = 5'000'000;

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};
   std::atomic<bool>          a_saw_b{false};

   auto bounded_spin = [&](std::atomic<std::uint64_t>& mine,
                           std::atomic<std::uint64_t>& peer,
                           std::atomic<bool>* saw_peer)
   {
      for (std::uint64_t i = 0; i < target; ++i) {
         mine.fetch_add(1, std::memory_order_relaxed);
         if (saw_peer && peer.load(std::memory_order_relaxed) != 0) {
            saw_peer->store(true);
         }
      }
   };

   // GIVEN:

   thread a(
      [&]{ bounded_spin(a_count, b_count, &a_saw_b); },
      a_stack,
      thread::priority(0),
      core0
   );

   thread b(
      [&]{ bounded_spin(b_count, a_count, nullptr); },
      b_stack,
      thread::priority(0),
      core0
   );

   // WHEN:

   kernel::start();

   // THEN:

   EXPECT_EQ(a_count.load(), target);
   EXPECT_EQ(b_count.load(), target);
   EXPECT_FALSE(a_saw_b.load())
      << "the first thread observed the second running before it finished, "
         "which means a preemption happened with no interrupt source present";

   kernel::finalise();
}


/* ============================================================================
 * Async preemption source: two non-yielding threads interleave
 *
 * A helper OS thread fires the reschedule IPI at core0 while two non-yielding
 * threads spin. Each thread watches the other's counter. Because neither yields,
 * the only way one can observe the other advancing is if an asynchronous signal
 * preempted it mid-loop and let the peer run. Observing that in both directions
 * proves the context relocation and signal-frame resume work end to end.
 * ========================================================================= */
TEST(SingleCorePreemption_Test,
     GivenTwoNonYieldingThreadsAndAnAsyncPreemptionSource_ThenTheyInterleave)
{
   kernel::initialise();

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};
   std::atomic<bool>          a_saw_b{false};
   std::atomic<bool>          b_saw_a{false};

   // Set true once a worker is running, so the helper knows the cores are up and
   // the port IPI is safe to call.
   std::atomic<bool> workers_active{false};

   // Set true by the helper when it is done firing. It both releases the workers
   // from their spin and guarantees no further IPI is sent.
   std::atomic<bool> helper_stopped{false};

   auto spin_worker = [&](std::atomic<std::uint64_t>& mine,
                          std::atomic<std::uint64_t>& peer,
                          std::atomic<bool>& saw_peer)
   {
      workers_active.store(true);

      // Spin with no yield at all. On a cooperative port the peer could never
      // run until we returned, so any observation of the peer advancing is proof
      // an asynchronous signal preempted us here.
      while (!helper_stopped.load()) {
         mine.fetch_add(1, std::memory_order_relaxed);
         if (peer.load(std::memory_order_relaxed) != 0) {
            saw_peer.store(true);
         }
      }
   };

   // GIVEN:

   thread a(
      [&]{ spin_worker(a_count, b_count, a_saw_b); },
      a_stack,
      thread::priority(0),
      core0
   );

   thread b(
      [&]{ spin_worker(b_count, a_count, b_saw_a); },
      b_stack,
      thread::priority(0),
      core0
   );

   // The interrupt source. It fires only while both workers are still spinning,
   // because helper_stopped is what releases them and it is set only after the
   // firing loop ends. So every IPI lands while the kernel is live.
   std::thread interrupt_source([&]{
      while (!workers_active.load()) {
         std::this_thread::yield();
      }

      // Bounded so a broken port fails the assertions below instead of hanging.
      constexpr int max_fires = 4000;
      for (int fired = 0; fired < max_fires; ++fired) {
         if (a_saw_b.load() && b_saw_a.load()) {
            break;
         }
         cyros_port_send_reschedule_ipi(0);
         std::this_thread::sleep_for(std::chrono::microseconds(50));
      }

      helper_stopped.store(true);
   });

   // WHEN:

   kernel::start();
   interrupt_source.join();

   // THEN:

   EXPECT_TRUE(a_saw_b.load())
      << "thread A never observed B advancing, so preemption did not interleave them";
   EXPECT_TRUE(b_saw_a.load())
      << "thread B never observed A advancing, so preemption did not interleave them";
   EXPECT_GT(a_count.load(), 0u);
   EXPECT_GT(b_count.load(), 0u);

   kernel::finalise();
}