/* ============================================================================
 * Priority inheritance: the machinery derived urgency introduced
 *
 * The other PI files observe that a holder's REPORTED priority changes. These
 * two observe things that only exist because urgency is computed at the pick
 * rather than cached and propagated, and that nothing else covers:
 *
 *   1. The pick itself. A boosted holder is no longer filed in the ready matrix
 *      under a boosted key, because the matrix is keyed on base priority and
 *      nothing re-keys it. It sits on its core's holder list and the pick folds
 *      urgency over that list to decide. So "the right thread runs" is now a
 *      DIFFERENT property from "get_priority reports the right number", and
 *      only the first one is what users experience. This test observes which
 *      thread actually runs.
 *
 *   2. Bridge overflow. A queue folds urgency over the waiters that themselves
 *      hold something, and snapshots a bounded number of them. Past that bound
 *      it deliberately reports maximum urgency, over-boosting the holder,
 *      because the alternative is under-reporting and unbounded inversion. That
 *      is a silent degradation path: nothing else in the suite reaches it, and
 *      if it inverted no existing test would notice.
 *
 * Same discipline as the sibling files: black box through lock/unlock and
 * get_priority, bounded polls with a bail path so a failure is an assertion
 * rather than a hang, and every spin gate waits on a value set from a
 * different core.
 * ========================================================================= */

#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;
static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

namespace
{

constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);
constexpr std::uint64_t poll_budget = 20'000'000;

struct alignas(CYROS_PORT_STACK_ALIGN) aligned_stack
{
   std::array<std::byte, STACK_SIZE> bytes;
};

template <typename Predicate>
[[nodiscard]] bool bounded_poll(Predicate&& done) noexcept
{
   for (std::uint64_t i = 0; i < poll_budget; ++i) {
      if (done()) return true;
      cyros_port_cpu_relax();
   }
   return false;
}

class SyncMutexPiDerived_Test : public ::testing::Test {};

}  // namespace


/* ============================================================================
 * The boost must change WHO RUNS, not merely what get_priority reports
 *
 * core0 holds two threads:
 *   H, base 5, takes the mutex and is then preempted
 *   N, base 3, a CPU-bound spinner that never blocks
 *
 * Under base-priority scheduling N wins forever: 3 beats 5, and with no slice
 * timer on this port a spinning N is never rotated out. H is on core0's holder
 * list, not in the ready matrix, so the ONLY way it ever runs again is the pick
 * folding its urgency and finding it beats N.
 *
 * Then U, base 1 on core2, blocks on the mutex. H's urgency becomes 1, and core0
 * must abandon N and run H.
 *
 * This is decisive rather than statistical. If the pick ignored the holder list,
 * or compared against the wrong key, or the donate path failed to prompt core0
 * to look again, H would never run at all and the poll would exhaust. There is
 * no interleaving in which the old base-priority-only pick passes this.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenHolderOutrankedByABusySpinner_WhenUrgentWaiterArrives_ThenTheHolderPreemptsIt)
{
   constexpr int reps = 20;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         mutex m;
         // N must BLOCK rather than spin until H owns the mutex. N has the
         // better base priority and shares H's core, so a spin here would let N
         // win the very first pick and starve H before it ever takes the lock,
         // which is the classic same-core-spinner deadlock this port invites.
         sync::semaphore n_gate{0};
         std::atomic<bool> h_holds{false};      // H owns the mutex
         std::atomic<bool> spinner_running{false};
         std::atomic<bool> h_resumed{false};    // H got the core back
         std::atomic<bool> stop_spinner{false};
         std::atomic<int>  failed_stage{0};
      };
      state s;

      // H: take the mutex, announce it, then spin until the spinner has taken
      // the core away. From that point H is READY, on core0's holder list, and
      // outranked on base priority.
      thread h(
         [&s]{
            s.m.lock();
            s.h_holds.store(true, std::memory_order_release);

            // Release N, which immediately preempts us: it is on this core with
            // the better base priority. From here we are READY and on core0's
            // holder list, outranked, and only a boost can get us back.
            s.n_gate.release();

            while (!s.spinner_running.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }

            // If we are ever scheduled again it is because the pick folded our
            // urgency and preferred us over N, which is the property under test.
            s.h_resumed.store(true, std::memory_order_release);
            s.stop_spinner.store(true, std::memory_order_release);
            s.m.unlock();
         },
         stacks[0].bytes, thread::priority(5), core0);

      // N: better base priority than H, never blocks. Becomes ready only once H
      // owns the mutex, so it cannot interfere with the acquisition itself.
      thread n(
         [&s]{
            s.n_gate.acquire();   // blocks, so H can run and take the mutex first
            s.spinner_running.store(true, std::memory_order_release);
            while (!s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
         },
         stacks[1].bytes, thread::priority(3), core0);

      // U: the urgent waiter, on another core so it can run while core0 is busy.
      thread u(
         [&s]{
            while (!s.spinner_running.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();     // donates urgency 1 to H
            s.m.unlock();
         },
         stacks[2].bytes, thread::priority(1), core2);

      // Conductor: bounded observation, and a bail that releases every gate so
      // the kernel can quiesce even when the property failed.
      thread driver(
         [&s]{
            if (!bounded_poll([&]{ return s.h_resumed.load(std::memory_order_acquire); })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
            }
         },
         stacks[3].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_EQ(s.failed_stage.load(), 0)
         << "the mutex holder never ran again: a boosted holder did not beat a "
            "better-base spinner on its own core";

      kernel::finalise();
   }
}


/* ============================================================================
 * Bridge overflow over-boosts, and must never under-boost
 *
 * A queue's urgency fold snapshots a bounded number of bridges, a bridge being
 * a waiter that itself holds something. Exceed that bound and the queue reports
 * 0, the most urgent value, deliberately: over-boosting costs fairness, whereas
 * under-boosting is the unbounded inversion the whole design exists to prevent.
 *
 * Build more bridges than the snapshot holds. Every B_i takes its own mutex and
 * then blocks on the shared one that C owns, so each is a bridge. C's reported
 * urgency must then be 0.
 *
 * The assertion is deliberately one-directional. The exact bound is an internal
 * constant and this test does not encode it: it asserts only that a pile of
 * bridges drives C to maximum urgency, and never that C is left at some
 * mid-value. If the bound is later raised, this still passes for the right
 * reason as long as enough bridges are built to exceed it.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenMoreBridgesThanTheSnapshotHolds_WhenTheyPileOnOneMutex_ThenTheHolderIsOverBoostedNotUnder)
{
   constexpr std::size_t bridges = 6;   // comfortably over any small snapshot bound
   constexpr int reps = 10;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, bridges + 2> stacks{};

      struct state
      {
         mutex shared;                          // the contended one, owned by C
         std::array<mutex, bridges> own{};      // one per bridge, so each holds something
         std::atomic<bool> c_holds{false};
         std::atomic<std::size_t> parked{0};    // bridges that have blocked on shared
         std::atomic<bool> release_c{false};
         std::atomic<int>  failed_stage{0};
         std::atomic<int>  observed{-1};
         thread* c{nullptr};
      };
      state s;

      thread c(
         [&s]{
            s.shared.lock();
            s.c_holds.store(true, std::memory_order_release);
            while (!s.release_c.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.shared.unlock();
         },
         stacks[0].bytes, thread::priority(20), core0);
      s.c = &c;   // the driver reads C's reported urgency through the public port

      // Each bridge: hold its own mutex, then block on the shared one. Base
      // priorities are mid-range and all EQUAL, so a correct bounded fold would
      // report that value and only the overflow path reports 0. That is what
      // makes the assertion discriminating rather than trivially satisfied.
      std::array<thread, bridges> b{};
      for (std::size_t i = 0; i < bridges; ++i) {
         b[i] = thread(
            [&s, i]{
               s.own[i].lock();
               while (!s.c_holds.load(std::memory_order_acquire)) {
                  cyros_port_cpu_relax();
               }
               s.parked.fetch_add(1, std::memory_order_acq_rel);
               s.shared.lock();
               s.shared.unlock();
               s.own[i].unlock();
            },
            stacks[i + 1].bytes,
            thread::priority(10),
            // Deliberately NOT core0. C spins there while holding the mutex, so
            // a bridge sharing that core would have the better base priority,
            // win the first pick, and spin on a flag C could never set. Bridges
            // spinning on cores 1 and 2 are safe because c_holds is set from a
            // different core, and each blocks straight afterwards, yielding to
            // its co-tenants.
            core_affinity::from_id(static_cast<std::uint32_t>(1 + (i % 2))));
      }

      thread driver(
         [&s]{
            if (!bounded_poll([&]{
                   return s.parked.load(std::memory_order_acquire) == bridges;
                })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.release_c.store(true, std::memory_order_release);
               return;
            }

            // Every bridge has announced it is about to block. Give the last
            // ones time to actually park and register as bridges before reading.
            if (!bounded_poll([&]{
                   return s.c->get_priority() == 0;
                })) {
               s.observed.store(s.c->get_priority(), std::memory_order_release);
               s.failed_stage.store(2, std::memory_order_release);
            }
            s.release_c.store(true, std::memory_order_release);
         },
         stacks[bridges + 1].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1) << "bridges never all parked on the shared mutex";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "holder was not driven to maximum urgency by an overflowing pile of "
            "bridges, it read " << s.observed.load()
         << ". Over-boosting is the safe direction here; anything else means the "
            "overflow path under-reports, which is unbounded inversion.";

      kernel::finalise();
   }
}
