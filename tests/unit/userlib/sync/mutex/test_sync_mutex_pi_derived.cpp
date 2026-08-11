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
 *   3. Ties. A holder's urgency can land exactly on the matrix's best base
 *      priority, and the two live in different structures with no arrival order
 *      between them. Whichever side loses a tie is not advanced by losing, so it
 *      loses the next one identically: a fixed policy is indefinite starvation,
 *      not unfairness. Two tests, one per structure boundary.
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
[[nodiscard]] bool bounded_poll(Predicate&& done, std::uint64_t budget = poll_budget) noexcept
{
   for (std::uint64_t i = 0; i < budget; ++i) {
      if (done()) return true;
      cyros_port_cpu_relax();
   }
   return false;
}

/* For a poll that is EXPECTED to exhaust in many rounds by design, like the
 * first-stage poll of the window-hunting test below, where a full budget per
 * exhaustion would dominate the suite's wall clock. Three orders of magnitude
 * above the microsecond scale of the event being awaited, and exhausting it
 * spuriously under load is harmless there because the full-budget second
 * stage still owns the verdict. */
constexpr std::uint64_t short_poll_budget = 250'000;

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


/* ============================================================================
 * A holder tied with a ready thread: both must keep getting turns
 *
 * core0 holds A at base 10 and H at base 20. H takes the mutex first, then
 * hands the core to A, which has the better base priority. H is now READY on
 * core0's holder list at urgency 20, and A legitimately wins every pick.
 *
 * D at base 10 on core1 then blocks on the mutex. H's urgency becomes exactly
 * 10, which is exactly A's key in the matrix. There is no arrival order between
 * the holder list and a matrix level, so the pick has to choose by policy.
 *
 * Both threads only ever yield, so neither can be starved by the other blocking
 * or by anything external: whoever stops making progress stopped because the
 * pick stopped choosing it. Being passed over does not change either one's
 * position, so a fixed tie policy does not merely skew the split, it stops one
 * side permanently.
 *
 * Fails in BOTH directions. Stage 1 catches ties going to the matrix forever,
 * which is starvation of a boosted holder, i.e. the inversion the whole
 * mechanism exists to prevent. Stage 2 catches ties going to the holder forever,
 * and measures A only AFTER the tie exists, since A runs freely before that and
 * an unqualified "did A run" would pass either way.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenAHolderTiedWithAReadyThread_WhenNeitherEverBlocks_ThenBothKeepGettingTurns)
{
   constexpr int reps = 10;
   constexpr unsigned progress = 5;   // turns each side must take while tied

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         mutex m;
         // A must not be runnable until H owns the mutex. A has the better base
         // priority and shares H's core, so it would otherwise win the first
         // pick and H would never acquire at all.
         sync::semaphore a_gate{0};
         std::atomic<bool> h_holds{false};
         std::atomic<unsigned> a_turns{0};
         std::atomic<unsigned> h_turns{0};
         std::atomic<bool> stop{false};
         std::atomic<int>  failed_stage{0};
      };
      state s;

      thread h(
         [&s]{
            s.m.lock();
            s.h_holds.store(true, std::memory_order_release);

            // Hands the core straight over: A is on this core with the better
            // base priority, so this preempts us here. Everything below runs
            // only if a later pick chose us, which is the property under test.
            s.a_gate.release();

            while (!s.stop.load(std::memory_order_acquire)) {
               s.h_turns.fetch_add(1, std::memory_order_acq_rel);
               this_thread::yield();
            }
            s.m.unlock();
         },
         stacks[0].bytes, thread::priority(20), core0);

      thread a(
         [&s]{
            s.a_gate.acquire();
            while (!s.stop.load(std::memory_order_acquire)) {
               s.a_turns.fetch_add(1, std::memory_order_acq_rel);
               this_thread::yield();
            }
         },
         stacks[1].bytes, thread::priority(10), core0);

      // The donor. Base 10, so it drives H's urgency onto exactly A's key.
      thread d(
         [&s]{
            while (!s.h_holds.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();
            s.m.unlock();
         },
         stacks[2].bytes, thread::priority(10), core1);

      thread driver(
         [&s]{
            if (!bounded_poll([&]{ return s.h_turns.load(std::memory_order_acquire) >= 1; })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.stop.store(true, std::memory_order_release);
               return;
            }

            // The tie is live and has been resolved at least once. From here
            // both sides must keep advancing.
            auto const a_mark = s.a_turns.load(std::memory_order_acquire);
            auto const h_mark = s.h_turns.load(std::memory_order_acquire);
            if (!bounded_poll([&]{
                   return s.a_turns.load(std::memory_order_acquire) >= a_mark + progress
                       && s.h_turns.load(std::memory_order_acquire) >= h_mark + progress;
                })) {
               s.failed_stage.store(2, std::memory_order_release);
            }
            s.stop.store(true, std::memory_order_release);
         },
         stacks[3].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1)
         << "a boosted holder tied with a ready thread never ran: the tie goes "
            "to the matrix every time, so the holder is starved and whoever is "
            "blocked on it waits without bound";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "one side stopped advancing while the tie was live (A took "
         << s.a_turns.load() << " turns, holder took " << s.h_turns.load()
         << "): the tie is not alternating";

      kernel::finalise();
   }
}


/* ============================================================================
 * Two holders tied with each other: both must keep getting turns
 *
 * The same defect one structure inwards. Nothing here is in core0's matrix at
 * all, so the matrix-versus-holder tie cannot fire and cannot mask this: the
 * only question is which of two equally urgent holders the fold picks.
 *
 * H1 and H2 sit on core0 holding a mutex each, both driven to urgency 10 by a
 * donor of their own on another core. The pick keeps the FIRST holder at the
 * best urgency, so the list's insertion order IS the tie policy. A holder is
 * unlinked when picked and re-linked when re-readied, so with head insertion
 * the thread that just ran goes back to the front and wins again, forever.
 *
 * The two holders have different base priorities purely so their startup order
 * is deterministic. Once both are boosted the bases are irrelevant, and that is
 * the point: they are tied on urgency, which is the only key the fold reads.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenTwoHoldersTiedAtEqualUrgency_WhenBothStayReady_ThenBothKeepGettingTurns)
{
   constexpr int reps = 10;
   constexpr unsigned progress = 5;
   constexpr std::uint8_t donor_priority = 10;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 5> stacks{};

      struct state
      {
         mutex m1;
         mutex m2;
         // Each holder parks until BOTH are boosted, so they enter the holder
         // list already tied. Acquiring in the clear keeps the setup free of
         // the very contention the test is about to create.
         sync::semaphore gate1{0};
         sync::semaphore gate2{0};
         std::atomic<bool> h1_holds{false};
         std::atomic<bool> h2_holds{false};
         std::atomic<unsigned> h1_turns{0};
         std::atomic<unsigned> h2_turns{0};
         std::atomic<bool> stop{false};
         std::atomic<int>  failed_stage{0};
         thread* h1{nullptr};
         thread* h2{nullptr};
      };
      state s;

      thread h1(
         [&s]{
            s.m1.lock();
            s.h1_holds.store(true, std::memory_order_release);
            s.gate1.acquire();
            while (!s.stop.load(std::memory_order_acquire)) {
               s.h1_turns.fetch_add(1, std::memory_order_acq_rel);
               this_thread::yield();
            }
            s.m1.unlock();
         },
         stacks[0].bytes, thread::priority(20), core0);

      thread h2(
         [&s]{
            s.m2.lock();
            s.h2_holds.store(true, std::memory_order_release);
            s.gate2.acquire();
            while (!s.stop.load(std::memory_order_acquire)) {
               s.h2_turns.fetch_add(1, std::memory_order_acq_rel);
               this_thread::yield();
            }
            s.m2.unlock();
         },
         stacks[1].bytes, thread::priority(21), core0);

      s.h1 = &h1;
      s.h2 = &h2;

      thread d1(
         [&s]{
            while (!s.h1_holds.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m1.lock();
            s.m1.unlock();
         },
         stacks[2].bytes, thread::priority(donor_priority), core1);

      thread d2(
         [&s]{
            while (!s.h2_holds.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m2.lock();
            s.m2.unlock();
         },
         stacks[3].bytes, thread::priority(donor_priority), core2);

      thread driver(
         [&s]{
            // Both donors parked, so both holders now report the donors'
            // priority. Only then are they tied.
            if (!bounded_poll([&]{
                   return s.h1->get_priority() == donor_priority
                       && s.h2->get_priority() == donor_priority;
                })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.gate1.release();
               s.gate2.release();
               return;
            }

            s.gate1.release();
            s.gate2.release();

            if (!bounded_poll([&]{
                   return s.h1_turns.load(std::memory_order_acquire) >= progress
                       && s.h2_turns.load(std::memory_order_acquire) >= progress;
                })) {
               s.failed_stage.store(2, std::memory_order_release);
            }
            s.stop.store(true, std::memory_order_release);
         },
         stacks[4].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1) << "the two holders never both reached equal urgency";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "one of two equally urgent holders never ran (h1 took "
         << s.h1_turns.load() << " turns, h2 took " << s.h2_turns.load()
         << "): the holder list hands every tie to the same thread, so its peer "
            "is starved and that peer's own waiter blocks without bound";

      kernel::finalise();
   }
}


/* ============================================================================
 * Urgency folds over EVERY held resource, and re-folds when one is given up
 *
 * donated_floor walks held_mask and takes the min across all of them. Every
 * other test in the suite has holders owning exactly one contended resource, so
 * a fold that stopped after the first occupied slot, or that latched the best
 * value it ever saw, would pass the entire suite.
 *
 * H holds two mutexes with waiters of different priorities. Two phases:
 *
 *   1. Both held. H must report the BETTER of the two waiters, not the first
 *      one found and not its own base.
 *   2. H releases the mutex carrying the better waiter. H must RISE to the
 *      remaining one, not stay at the value it had. Nothing cached the boost,
 *      so there is nothing to un-cache, and this is the test that says so.
 *
 * Phase 2 is the half that catches a latch. Phase 1 alone would pass against an
 * implementation that remembered its best-ever donation forever.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenTwoHeldMutexesWithDifferentWaiters_WhenTheBetterOneIsReleased_ThenUrgencyRisesToTheOther)
{
   constexpr int reps = 10;
   constexpr std::uint8_t mild_priority = 10;
   constexpr std::uint8_t keen_priority = 4;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         mutex mild;   // waiter at 10
         mutex keen;   // waiter at 4, the better donation
         std::atomic<bool> h_holds_both{false};
         std::atomic<bool> drop_keen{false};
         std::atomic<bool> drop_mild{false};
         std::atomic<int>  failed_stage{0};
         std::atomic<int>  observed{-1};
         thread* h{nullptr};
      };
      state s;

      thread h(
         [&s]{
            s.mild.lock();
            s.keen.lock();
            s.h_holds_both.store(true, std::memory_order_release);

            while (!s.drop_keen.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.keen.unlock();

            while (!s.drop_mild.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.mild.unlock();
         },
         stacks[0].bytes, thread::priority(20), core0);
      s.h = &h;

      auto const waiter = [&s](mutex& m) {
         return [&s, &m]{
            while (!s.h_holds_both.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            m.lock();
            m.unlock();
         };
      };

      thread w_mild(waiter(s.mild), stacks[1].bytes, thread::priority(mild_priority), core1);
      thread w_keen(waiter(s.keen), stacks[2].bytes, thread::priority(keen_priority), core2);

      thread driver(
         [&s]{
            // Phase 1: the min across BOTH held resources.
            if (!bounded_poll([&]{ return s.h->get_priority() == keen_priority; })) {
               s.observed.store(s.h->get_priority(), std::memory_order_release);
               s.failed_stage.store(1, std::memory_order_release);
               s.drop_keen.store(true, std::memory_order_release);
               s.drop_mild.store(true, std::memory_order_release);
               return;
            }

            // Phase 2: give up the better one, keep the other.
            s.drop_keen.store(true, std::memory_order_release);
            if (!bounded_poll([&]{ return s.h->get_priority() == mild_priority; })) {
               s.observed.store(s.h->get_priority(), std::memory_order_release);
               s.failed_stage.store(2, std::memory_order_release);
            }
            s.drop_mild.store(true, std::memory_order_release);
         },
         stacks[3].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1)
         << "holder of two contended mutexes did not report the better waiter, it read "
         << s.observed.load() << " (expected " << int(keen_priority)
         << "). The fold is not covering every occupied held slot.";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "after releasing the mutex carrying the better waiter the holder read "
         << s.observed.load() << " (expected " << int(mild_priority)
         << "). Urgency is being latched rather than re-derived from what is still held.";

      kernel::finalise();
   }
}


/* ============================================================================
 * A holder that acquired by HANDOVER must be boostable too
 *
 * The sibling test above has the holder take the mutex uncontended, so it is
 * registered in held_slots by its own acquiring poll. This one has it PARK on a
 * held mutex and receive ownership from the releaser instead, which is the other
 * way a thread becomes a holder and the one where ownership is committed by a
 * different core.
 *
 * That commit runs under the queue lock, where the new owner's pi_lock is out of
 * reach, so it is the case where "file the resource in the owner's slots" is
 * hard rather than obvious. It has to happen anyway: routing at set_thread_ready
 * and the urgency fold both key on held_slots, so ownership that is not there is
 * ownership nothing can boost. H would sit in the ready matrix at base priority
 * behind N for as long as N cares to spin, with U parked on the mutex H owns,
 * which is precisely the unbounded inversion the subsystem exists to prevent.
 *
 * The choreography is otherwise the sibling's: N outranks H on base priority and
 * never blocks, U is urgent and arrives only after the handover, and H running
 * again at all is the property.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenAHolderThatAcquiredByHandover_WhenAnUrgentWaiterArrives_ThenItStillPreemptsTheSpinner)
{
   constexpr int reps = 20;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 6> stacks{};

      struct state
      {
         mutex m;
         sync::semaphore n_gate{0};
         std::atomic<bool> l_holds{false};        // L owns the mutex
         std::atomic<bool> h_parked{false};       // the witness saw H park
         std::atomic<bool> spinner_running{false};
         std::atomic<bool> l_released{false};     // ownership handed to H
         std::atomic<bool> h_resumed{false};      // H got the core back
         std::atomic<bool> stop_spinner{false};
         std::atomic<int>  failed_stage{0};
         std::atomic<int>  h_urgency{-1};         // what H reported when it stuck
         thread* h{nullptr};
      };
      state s;

      // L, on its own core: takes the mutex first so H must park on it and be
      // handed ownership rather than taking it uncontended.
      thread l(
         [&s]{
            s.m.lock();
            s.l_holds.store(true, std::memory_order_release);
            // Hold until the spinner owns core0, so the handover readies H
            // BEHIND a better-base thread instead of straight onto the core.
            while (!s.stop_spinner.load(std::memory_order_acquire)
                   && !s.spinner_running.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.unlock();                                  // hands the mutex to H
            s.l_released.store(true, std::memory_order_release);
         },
         stacks[0].bytes, thread::priority(4), core1);

      thread h(
         [&s]{
            while (!s.l_holds.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();
            s.h_resumed.store(true, std::memory_order_release);
            s.stop_spinner.store(true, std::memory_order_release);
            s.m.unlock();
         },
         stacks[1].bytes, thread::priority(5), core0);
      s.h = &h;

      // Witness: worse base priority than H and on H's core, so it runs only
      // once H has actually parked. Releasing the spinner from anywhere else
      // races H's park and can starve H before it ever reaches the queue.
      thread w(
         [&s]{
            s.h_parked.store(true, std::memory_order_release);
            s.n_gate.release();
         },
         stacks[2].bytes, thread::priority(6), core0);

      // N: better base priority than H, never blocks, shares H's core.
      thread n(
         [&s]{
            s.n_gate.acquire();
            s.spinner_running.store(true, std::memory_order_release);
            while (!s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
         },
         stacks[3].bytes, thread::priority(3), core0);

      // U: the urgent waiter. It must arrive AFTER the handover, otherwise it
      // would be the best waiter and the release would choose it, not H.
      thread u(
         [&s]{
            while (!s.l_released.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();     // donates urgency 1 to H
            s.m.unlock();
         },
         stacks[4].bytes, thread::priority(1), core2);

      thread driver(
         [&s]{
            if (!bounded_poll([&]{ return s.h_parked.load(std::memory_order_acquire); })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
               return;
            }
            if (!bounded_poll([&]{ return s.h_resumed.load(std::memory_order_acquire); })) {
               // Reported urgency separates the two ways this can fail: base
               // priority means the donation never reached H at all, urgency 1
               // means it did and the pick ignored it.
               s.h_urgency.store(s.h->get_priority(), std::memory_order_release);
               s.failed_stage.store(2, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
            }
         },
         stacks[5].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1) << "H never parked on the held mutex";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "a holder that acquired the mutex by handover never ran again, and "
            "reported urgency " << s.h_urgency.load()
         << ". Ownership committed by the releaser is not reaching held_slots, so "
            "nothing can boost it.";

      kernel::finalise();
   }
}


/* ============================================================================
 * A transitive boost has to PROMPT the far core, not just be computable there
 *
 * Chain: A -> m2 (held by B) -> B -> m1 (held by C) -> C.
 *
 * The chain tests elsewhere check that every link's urgency is COMPUTED
 * correctly, and they check it by polling get_priority from threads that yield
 * constantly, which manufactures reschedules on every core. That hides the
 * question this test asks: does anything actually tell C's core to look?
 *
 * Priorities make the two donations distinguishable:
 *   A 1, N 2, B 3, C 5
 * B's DIRECT donation puts C at urgency 3, which still loses to the spinner N
 * at base 2. Only the TRANSITIVE donation from A puts C at 1, which wins. So C
 * running again is proof the second-order boost both computed AND arrived, and
 * N never yields, so nothing else can hand C the core.
 *
 * The bail path records C's REPORTED urgency, which is what makes a failure
 * diagnostic rather than merely red: 1 means the fold is right and the prompt
 * never arrived, anything else means the fold itself is wrong.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenATwoLinkChainAcrossCores_WhenTheUrgentWaiterParks_ThenTheFarHolderStillPreempts)
{
   constexpr int reps = 5;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 6> stacks{};

      struct state
      {
         mutex m1;                              // held by C, wanted by B
         mutex m2;                              // held by B, wanted by A
         sync::semaphore n_gate{0};
         std::atomic<bool> c_holds{false};      // C owns m1
         std::atomic<bool> b_parked{false};     // witness saw B park on m1
         std::atomic<bool> c_resumed{false};    // C got core0 back: THE PROPERTY
         std::atomic<bool> a_done{false};       // A finally acquired m2
         std::atomic<bool> stop_spinner{false};
         std::atomic<int>  failed_stage{0};
         std::atomic<int>  c_urgency{-1};
         std::atomic<int>  b_urgency{-1};
         thread* b{nullptr};
         thread* c{nullptr};
      };
      state s;

      // C, base 5 on core0: take m1, then hand the core to N and do not come
      // back until something decides C is more urgent than N.
      thread c(
         [&s]{
            s.m1.lock();
            s.c_holds.store(true, std::memory_order_release);
            s.n_gate.release();          // N (base 2) preempts us right here
            s.c_resumed.store(true, std::memory_order_release);
            s.stop_spinner.store(true, std::memory_order_release);
            s.m1.unlock();
         },
         stacks[0].bytes, thread::priority(5), core0);
      s.c = &c;

      // N, base 2 on core0: never blocks. Beats C's base 5 and beats the
      // urgency 3 that B's direct donation produces. Loses to 1.
      thread n(
         [&s]{
            s.n_gate.acquire();
            while (!s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
         },
         stacks[1].bytes, thread::priority(2), core0);

      // B, base 3 on core1: the middle link. Holds m2, blocks on m1, and is
      // therefore a BRIDGE on m1's queue.
      thread b(
         [&s]{
            s.m2.lock();
            while (!s.c_holds.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m1.lock();      // parks behind C, donating urgency 3
            s.m1.unlock();
            s.m2.unlock();
         },
         stacks[2].bytes, thread::priority(3), core1);
      s.b = &b;

      // Witness on B's core, worse base priority, so it runs only once B has
      // actually parked on m1.
      thread wb(
         [&s]{
            s.b_parked.store(true, std::memory_order_release);
         },
         stacks[3].bytes, thread::priority(6), core1);

      // A, base 1 on core2: arrives last and should lift C to 1 THROUGH B.
      thread a(
         [&s]{
            while (!s.b_parked.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m2.lock();
            s.m2.unlock();
            s.a_done.store(true, std::memory_order_release);
         },
         stacks[4].bytes, thread::priority(1), core2);

      thread driver(
         [&s]{
            if (!bounded_poll([&]{ return s.b_parked.load(std::memory_order_acquire); })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
               return;
            }
            if (!bounded_poll([&]{ return s.c_resumed.load(std::memory_order_acquire); })) {
               s.c_urgency.store(s.c->get_priority(), std::memory_order_release);
               s.b_urgency.store(s.b->get_priority(), std::memory_order_release);
               s.failed_stage.store(2, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
            }
         },
         stacks[5].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1) << "B never parked on m1";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "the far holder C never ran again. C reported urgency "
         << s.c_urgency.load() << ", B reported " << s.b_urgency.load()
         << " (C at 1 means the fold is right and nothing prompted core0)";

      kernel::finalise();
   }
}


/* ============================================================================
 * The prompt walk must not stop at a waiter that is not parked yet
 *
 * Same chain as the sibling test above: A -> m2 (held by B) -> B -> m1 (held
 * by C) -> C, with a spinner N on C's core that only urgency 1 beats. The
 * sibling parks B before A donates, so the walk always meets B blocked. This
 * one aims A's donation at the window between B arming on m1 and B parking.
 * In that window B is still running, or already rotated out READY by a
 * preemption with blocked_on still published, and it stays ready for as long
 * as a better thread wants its core. A donation arriving then must still walk
 * through B to C. B cannot pass it on itself once its own donate has run, it
 * will not run again to repeat it, and C's core has no tick to save it.
 *
 * Two independent hitters sweep that window from the b_locking anchor B
 * publishes just before locking. A jitters its own donation so its walk reads
 * B mid-window while B is running-and-armed. The driver jitters the release
 * of P, base 0 on B's core, whose wake IPI preempts B mid-window and holds it
 * off the core, so a later walk reads B ready-and-armed. The window is
 * microseconds wide and the anchor-to-hit latencies are comparable, so not
 * every round catches it. Rounds that miss degenerate to the sibling's
 * blocked-B case and still assert the walk worked.
 *
 * The bail path is two-staged for a reason. If C never resumes, the driver
 * first stops P alone and polls again. A round where B was preempted before
 * its own donate ran self-heals here, because the resumed B finishes its
 * acquire poll and prompts C itself, and that is a pass. A round where the
 * walk genuinely stopped at a not-yet-blocked B does not self-heal, B just
 * parks, and only then is the failure recorded. That distinction is what
 * makes a red run diagnostic: C reporting urgency 1 at the failure means the
 * fold computed the transitive boost and nothing delivered the prompt.
 * ========================================================================= */
TEST_F(SyncMutexPiDerived_Test,
       GivenATwoLinkChainAcrossCores_WhenTheMiddleWaiterIsNotParkedYet_ThenTheFarHolderStillPreempts)
{
   /* 140 rather than the 65 the sweep needs for coverage: detection is
    * statistical here, unlike the deterministic mutation tests elsewhere, and
    * the measured per-round catch against the reinstated gate put a 65-round
    * run at roughly 75 percent. Doubling the rounds was measured, not assumed,
    * to move a single run into the mid-90s. cross-core-defects.md section 9
    * has the numbers. */
   constexpr int reps = 140;

   for (int rep = 0; rep < reps; ++rep) {
      SCOPED_TRACE("rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 6> stacks{};

      struct state
      {
         mutex m1;                              // held by C, wanted by B
         mutex m2;                              // held by B, wanted by A
         sync::semaphore n_gate{0};
         sync::semaphore p_gate{0};
         std::atomic<bool> c_holds{false};      // C owns m1
         std::atomic<bool> spinner_running{false};
         std::atomic<bool> b_locking{false};    // B is about to arm on m1
         std::atomic<bool> c_resumed{false};    // C got core0 back: THE PROPERTY
         std::atomic<bool> stop_spinner{false};
         std::atomic<bool> stop_p{false};
         std::atomic<int>  failed_stage{0};
         std::atomic<int>  c_urgency{-1};
         std::atomic<int>  b_urgency{-1};
         thread* b{nullptr};
         thread* c{nullptr};
      };
      state s;

      // C, base 5 on core0: take m1, hand the core to N, and come back only
      // when something decides C is more urgent than N.
      thread c(
         [&s]{
            s.m1.lock();
            s.c_holds.store(true, std::memory_order_release);
            s.n_gate.release();          // N (base 2) preempts us right here
            s.c_resumed.store(true, std::memory_order_release);
            s.stop_spinner.store(true, std::memory_order_release);
            s.m1.unlock();
         },
         stacks[0].bytes, thread::priority(5), core0);
      s.c = &c;

      // N, base 2 on core0: never blocks. Beats C's base 5 and the urgency 3
      // of B's direct donation. Loses only to the transitive 1.
      thread n(
         [&s]{
            s.n_gate.acquire();
            s.spinner_running.store(true, std::memory_order_release);
            while (!s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
         },
         stacks[1].bytes, thread::priority(2), core0);

      // B, base 3 on core1: the middle link. Waits until N owns core0, so its
      // donation cannot land while C is still running and pass the round
      // vacuously, then announces the lock attempt and arms.
      thread b(
         [&s, rep]{
            s.m2.lock();
            while (!s.spinner_running.load(std::memory_order_acquire)
                   && !s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.b_locking.store(true, std::memory_order_release);
            // Bounded stall, swept on a stride coprime with the driver's, so
            // the arm-to-park window slides through P's fairly fixed wake
            // latency and the two meet at some alignment in every run.
            for (int j = (rep % 13) * 96; j > 0; --j) {
               cyros_port_cpu_relax();
            }
            s.m1.lock();
            s.m1.unlock();
            s.m2.unlock();
         },
         stacks[2].bytes, thread::priority(3), core1);
      s.b = &b;

      // P, base 0 on B's core: released mid-window, then holds B off the core
      // so B stays ready-and-armed while A donates.
      thread p(
         [&s]{
            s.p_gate.acquire();
            while (!s.stop_p.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
         },
         stacks[3].bytes, thread::priority(0), core1);

      // A, base 1 on core2: the donor. Fires at a fixed delay past every
      // possible landing of P, so in a round where P caught the window the
      // walk is guaranteed to read a B that is READY, still armed, and whose
      // own donate has already run. Firing earlier would not strengthen the
      // test, it would let A's arm on m2 land before B's donate, and B's
      // donate would then carry A's urgency to C by the fold and rescue a
      // swallowed walk before it was ever observable.
      thread a(
         [&s]{
            while (!s.b_locking.load(std::memory_order_acquire)
                   && !s.stop_spinner.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            for (int j = 8192; j > 0; --j) {
               cyros_port_cpu_relax();
            }
            s.m2.lock();
            s.m2.unlock();
         },
         stacks[4].bytes, thread::priority(1), core2);

      thread driver(
         [&s, rep]{
            if (!bounded_poll([&]{ return s.b_locking.load(std::memory_order_acquire); })) {
               s.failed_stage.store(1, std::memory_order_release);
               s.stop_p.store(true, std::memory_order_release);
               s.stop_spinner.store(true, std::memory_order_release);
               s.p_gate.release();
               return;
            }

            // Sweep P's release across B's arm-to-park window round by round.
            // A hit rotates B out ready-and-armed and holds it there for A.
            for (int j = (rep % 5) * 128; j > 0; --j) {
               cyros_port_cpu_relax();
            }
            s.p_gate.release();

            if (bounded_poll([&]{ return s.c_resumed.load(std::memory_order_acquire); },
                             short_poll_budget)) {
               s.stop_p.store(true, std::memory_order_release);
               return;
            }

            // Stop P alone first, then wait the full budget for the verdict.
            // A round where B was preempted before its own donate ran
            // self-heals now, a genuinely swallowed walk does not.
            s.stop_p.store(true, std::memory_order_release);
            if (bounded_poll([&]{ return s.c_resumed.load(std::memory_order_acquire); })) {
               return;
            }

            s.c_urgency.store(s.c->get_priority(), std::memory_order_release);
            s.b_urgency.store(s.b->get_priority(), std::memory_order_release);
            s.failed_stage.store(2, std::memory_order_release);
            s.stop_spinner.store(true, std::memory_order_release);
         },
         stacks[5].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_NE(s.failed_stage.load(), 1) << "B never reached its lock of m1";
      EXPECT_NE(s.failed_stage.load(), 2)
         << "the far holder C never ran again with the middle waiter unparked. "
            "C reported urgency " << s.c_urgency.load() << ", B reported "
         << s.b_urgency.load()
         << " (C at 1 means the fold is right and the walk stopped at a "
            "not-yet-blocked B)";

      kernel::finalise();
   }
}
