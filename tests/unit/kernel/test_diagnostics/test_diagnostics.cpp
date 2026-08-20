/**
 * @file test_diagnostics.cpp
 * @brief The blocking-chain query on live, resolvable chains.
 *
 * Every chain built here resolves and every thread terminates, so the suite
 * ends by quiescing normally. The wait-for CYCLE case cannot be tested that
 * way, a deadlocked thread never exits, so it lives in its own binary
 * (test_diagnostics_deadlock) that ends the process explicitly.
 *
 * Single core. The interleaving is driven by priorities and semaphore gates:
 * a release of a more urgent thread preempts the releaser at its next
 * scheduling point, which is how each thread is walked into position.
 */

#include <cyros/kernel/diagnostics.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/thread.hpp>
#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>

using namespace cyros;

namespace
{

alignas(16) std::array<std::byte, 64 * 1024> s_a;
alignas(16) std::array<std::byte, 64 * 1024> s_b;
alignas(16) std::array<std::byte, 64 * 1024> s_c;
alignas(16) std::array<std::byte, 64 * 1024> s_d;

class Diagnostics_Test : public ::testing::Test
{
protected:
   void SetUp() override    { kernel::initialise(); }
   void TearDown() override { kernel::finalise(); }
};

} // namespace

TEST_F(Diagnostics_Test, GivenAnUnblockedThread_WhenQueried_ThenTheChainIsEmpty)
{
   diagnostics::blocking_chain chain;
   chain.deadlocked = true; // prove the call clears it

   thread idle_thread([]{}, s_a, thread::priority(1), core0);

   thread querier(
      [&]{
         chain = diagnostics::blocking_chain_of(idle_thread);
      },
      s_b, thread::priority(2), core0
   );

   kernel::start();

   EXPECT_EQ(chain.length, 0u);
   EXPECT_FALSE(chain.deadlocked);
}

TEST_F(Diagnostics_Test, GivenATwoHopChain_WhenQueriedFromTheTail_ThenBothHoldersAppearInOrder)
{
   /* One struct captured by one reference, because thread::entry_fn stores
    * its callable inline with no heap fallback. */
   struct
   {
      sync::mutex m1;
      sync::mutex m2;
      sync::semaphore gate_b{0};
      sync::semaphore gate_c{0};
      thread* pa{nullptr};
      thread* pb{nullptr};
      thread* pc{nullptr};
      diagnostics::blocking_chain of_c;
      diagnostics::blocking_chain of_b;
      diagnostics::blocking_chain of_a_self;
   } t;

   /* Target shape, built inside-out on one core:
    *   C blocked on m1, held by B, blocked on m2, held by A, running.
    * C and B park on gates first so the least urgent thread A can take m2 and
    * then walk them into position by releasing the gates. */
   thread c(
      [&t]{
         t.gate_c.acquire();
         t.m1.lock();     // Blocks behind B until the chain resolves
         t.m1.unlock();
      },
      s_c, thread::priority(1), core0
   );

   thread b(
      [&t]{
         t.gate_b.acquire();
         t.m1.lock();
         t.gate_c.release(); // C preempts, blocks on m1
         t.m2.lock();        // Blocks behind A. Back to A, the only runnable
         t.m2.unlock();
         t.m1.unlock();
      },
      s_b, thread::priority(2), core0
   );

   thread a(
      [&t]{
         t.m2.lock();
         t.gate_b.release(); // B preempts, takes m1, wakes C, blocks on m2

         // Chain is staged: C -> m1 -> B -> m2 -> A(this, running).
         t.of_c      = diagnostics::blocking_chain_of(*t.pc);
         t.of_b      = diagnostics::blocking_chain_of(*t.pb);
         t.of_a_self = diagnostics::blocking_chain_of(*t.pa);

         t.m2.unlock(); // Resolves the whole chain
      },
      s_a, thread::priority(5), core0
   );

   t.pa = &a;
   t.pb = &b;
   t.pc = &c;

   kernel::start();

   ASSERT_EQ(t.of_c.length, 2u);
   EXPECT_EQ(t.of_c.holders[0], b.get_id());
   EXPECT_EQ(t.of_c.holders[1], a.get_id());
   EXPECT_FALSE(t.of_c.deadlocked);

   ASSERT_EQ(t.of_b.length, 1u);
   EXPECT_EQ(t.of_b.holders[0], a.get_id());
   EXPECT_FALSE(t.of_b.deadlocked);

   EXPECT_EQ(t.of_a_self.length, 0u); // A runs, so A is blocked behind nobody
}

TEST_F(Diagnostics_Test, GivenAResolvedChain_WhenQueriedAgain_ThenItReadsEmpty)
{
   struct
   {
      sync::mutex m;
      sync::semaphore gate{0};
      thread* waiter{nullptr};
      diagnostics::blocking_chain during;
      diagnostics::blocking_chain after;
   } t;

   thread waiter(
      [&t]{
         t.gate.acquire();
         t.m.lock();
         t.m.unlock();
      },
      s_b, thread::priority(1), core0
   );

   thread holder(
      [&t]{
         t.m.lock();
         t.gate.release();                                   // waiter preempts, blocks on m
         t.during = diagnostics::blocking_chain_of(*t.waiter); // chain exists
         t.m.unlock();                                         // waiter takes it and finishes
         t.after = diagnostics::blocking_chain_of(*t.waiter);  // chain gone
      },
      s_a, thread::priority(5), core0
   );

   t.waiter = &waiter;

   kernel::start();

   ASSERT_EQ(t.during.length, 1u);
   EXPECT_EQ(t.during.holders[0], holder.get_id());
   EXPECT_EQ(t.after.length, 0u);
}

TEST_F(Diagnostics_Test, GivenChurningHandovers_WhenQueriedContinuously_ThenNoPhantomDeadlockIsReported)
{
   /* Two threads trading one mutex present exactly the stale shapes a naive
    * detector misreads: between a handover and the new owner running, a walk
    * can see the mutex's owner and its own start node as the same thread, or
    * see both threads pointing through the same mutex. None of these is a
    * deadlock, and the confirmation re-read must reject every one. Under the
    * preempt port the observer's walk races real handovers.
    *
    * This is the negative test for confirm_cycle: weaken it to a single-pass
    * report and this asserts within a few thousand queries. */
   struct
   {
      sync::mutex m1;
      sync::mutex m2;
      std::atomic<bool> stop{false};
      thread* p1{nullptr};
      thread* p2{nullptr};
      thread* p3{nullptr};
      std::atomic<std::uint32_t> phantom{0};
      std::atomic<std::uint32_t> queries{0};
   } t;

   /* Same-order nested locking, so a REAL cycle is impossible, while stale
    * cross-iteration reads can still stitch one: the walk can hold last
    * round's owner of m2 while that thread is already blocked on m1 in its
    * next round, which reads as m2-holder-waiting-on-m1, the edge a genuine
    * deadlock would need. Yielding while holding is what makes anyone block
    * at all on a tickless single core. */
   auto churn = [&t]{
      while (!t.stop.load(std::memory_order_relaxed)) {
         t.m1.lock();
         this_thread::yield();
         t.m2.lock();
         this_thread::yield();
         t.m2.unlock();
         t.m1.unlock();
         this_thread::yield();
      }
   };

   /* Workers share core 0, the observer runs on core 1. On one core the
    * probe is blind by construction: with no tick, whatever state exists when
    * the observer is picked is frozen until it yields, and the deterministic
    * rotation closes every handover window before the observer ever runs.
    * Measured, not supposed: 40,000 single-core walks observed blocked_on set
    * ZERO times. A second core samples the window concurrently instead. */
   thread t1(churn, s_a, thread::priority(1), core0);
   thread t2(churn, s_b, thread::priority(1), core0);
   thread t3(churn, s_d, thread::priority(1), core0);

   thread observer(
      [&t]{
         for (std::uint32_t i = 0; i < 20'000; ++i) {
            if (diagnostics::blocking_chain_of(*t.p1).deadlocked ||
                diagnostics::blocking_chain_of(*t.p2).deadlocked ||
                diagnostics::blocking_chain_of(*t.p3).deadlocked) {
               t.phantom.fetch_add(1, std::memory_order_relaxed);
            }
            t.queries.fetch_add(1, std::memory_order_relaxed);
            this_thread::yield();
         }
         t.stop.store(true, std::memory_order_relaxed);
      },
      s_c, thread::priority(1), core1
   );

   t.p1 = &t1;
   t.p2 = &t2;
   t.p3 = &t3;

   kernel::start();

   EXPECT_EQ(t.phantom.load(), 0u);
   EXPECT_EQ(t.queries.load(), 20'000u);
}
