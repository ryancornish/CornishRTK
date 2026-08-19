#include <cyros/kernel/kernel.hpp>
#include <cyros/kernel/waitable.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include <common/guarded_stack.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

namespace
{

// Tune this knob to trade test speed for race-detection confidence.
// 200 catches most races with reasonable test runtime; bump for harder
// hunting, drop when iterating on unrelated work.
constexpr int stress_repetitions = 200;

// Each test owns its own kernel lifecycle per rep, so the fixture is empty.
class MultiCoreTransfer_Test : public ::testing::Test {};

/**
 * @brief Minimal barge-free resource built on waitable::wake_one_and_transfer.
 *
 * Deliberately mirrors the shape a real priority_mutex would use: a single
 * atomic owner id, 0 meaning free, taken by CAS when uncontended and passed
 * by ownership on release when contended. try_satisfy() recognises BOTH the
 * uncontended take and "ownership was already transferred to me while parked",
 * which is what lets a wake_one_and_transfer-released resource still compose with
 * wait_on / wait_on_any.
 *
 * release() exposes wake_one_and_transfer() directly (protected in waitable) so test
 * threads can drive it without protected access.
 */
class TransferResource final : public waitable
{
public:
   std::atomic<std::uint32_t> owner{0};

   // Non-blocking take-if-free, for tests that want to seed contention
   // without a thread parking first.
   bool try_acquire() noexcept
   {
      std::uint32_t expected = 0;
      return owner.compare_exchange_strong(expected, this_thread::id(),
                                           std::memory_order_acq_rel);
   }

   // Barge-free release: transfer ownership to the best-priority waiter, or free
   // the resource if none is parked. Mirrors priority_mutex::unlock().
   void release() noexcept
   {
      wake_one_and_transfer([this](std::uint32_t next_owner_id) {
         owner.store(next_owner_id, std::memory_order_release);
      });
   }

   // wake_one() is protected on waitable; expose it so a test can prove that
   // driving the ordinary barge-permitting wake on a wake_one_and_transfer-style resource
   // is a MISUSE this suite explicitly avoids relying on. Not used by the
   // transfer tests below; present for the barge-comparison test only.
   void barge_wake_one() noexcept { wake_one(); }

protected:
   bool try_satisfy() noexcept override
   {
      std::uint32_t expected = 0;
      if (owner.compare_exchange_strong(expected, this_thread::id(), std::memory_order_acq_rel)) {
         return true; // free, uncontended take
      }
      // ownership may already have been handed to us by a racing release().
      return expected == this_thread::id();
   }
};

}  // namespace

/* ============================================================================
 * Single-core ownership correctness
 *
 * Establishes the basic contract on one core before adding cross-core
 * timing: a release with a waiter present hands ownership directly to that
 * waiter, and the waiter observes itself as owner without re-taking.
 * ========================================================================= */
TEST_F(MultiCoreTransfer_Test,
       GivenOneWaiterParked_WhenOwnerReleases_ThenWaiterIsTransferredOwnershipDirectly)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack owner_stack;
      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack witness_stack;

      TransferResource r;
      std::atomic<bool> owner_ready{false};
      std::atomic<bool> waiter_armed{false};
      std::atomic<std::uint32_t> waiter_id{0};
      std::atomic<std::uint32_t> observed_owner{0xFFFF'FFFFu};

      thread owner_thread(
         [&]{
            // Must win the first CAS deterministically: try_satisfy()'s
            // uncontended-take branch is the SAME CAS try_acquire() performs,
            // so without this gate a waiter that reaches wait_on(r) first
            // would take r itself and this ASSERT would fail.
            ASSERT_TRUE(r.try_acquire());
            owner_ready.store(true, std::memory_order_release);
            // Release only once waiter_armed proves the waiter has genuinely
            // armed and parked (see waiter_block_witness below), not merely
            // signalled intent to. Arming happens inside wait_on, invisible
            // to a flag set before calling it; releasing too early would
            // take the free branch instead of the transfer branch this test
            // exists to exercise.
            while (!waiter_armed.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            r.release();
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      thread waiter(
         [&]{
            while (!owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            waiter_id.store(this_thread::id(), std::memory_order_release);
            this_thread::wait_on(r);
            // On return, ownership must already be ours: no re-take race.
            observed_owner.store(r.owner.load(std::memory_order_acquire), std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core1
      );

      // Pinned to the SAME core as waiter, at a strictly LOWER priority.
      // Priority scheduling on a single core is strict, so this thread
      // cannot run until waiter transitions out of ready/running - which,
      // given the two-phase block protocol (arm under the queue lock, then
      // commit to blocked), can only happen once waiter has genuinely armed
      // on r's queue and parked. The instant this runs is proof of that,
      // via a single atomic store with no lock touched at all, unlike
      // polling the waitable's internal queue state directly (which would
      // contend the very spinlock arm() needs).
      thread waiter_block_witness(
         [&]{ waiter_armed.store(true, std::memory_order_release); },
         witness_stack,
         thread::priority(1),
         core1
      );

      kernel::start();

      ASSERT_NE(waiter_id.load(), 0u);
      EXPECT_EQ(observed_owner.load(), waiter_id.load());
      kernel::finalise();
   }
}

/* ============================================================================
 * Barge-freedom under cross-core contention
 *
 * The property wake_one_and_transfer exists for: while a thread is parked waiting for
 * the resource, a FRESH thread on another core that never parked must not be
 * able to steal it out from under the parked waiter. The fresh thread races
 * try_acquire() against the owner's release() from a different core; the
 * parked waiter must win every time, because the release commits ownership
 * under the same lock that a racing try_acquire's CAS would need to observe
 * the resource as free.
 *
 * If barging occurred, the parked waiter would remain blocked forever (its
 * try_satisfy() would keep failing, since the resource was taken by
 * someone else) and the test would hang / time out rather than fail cleanly,
 * so this is run under the stress loop to build confidence across many
 * timing windows rather than relying on a single lucky/unlucky interleaving.
 * ========================================================================= */
TEST_F(MultiCoreTransfer_Test,
       GivenWaiterParkedAndFreshCoreRacingAcquire_WhenOwnerReleases_ThenParkedWaiterWinsNotBarger)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack owner_stack;
      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack witness_stack;
      cyros::test::guarded_stack barger_stack;

      TransferResource r;
      std::atomic<bool> owner_ready{false};
      std::atomic<bool> waiter_armed{false};
      std::atomic<bool> barger_spinning{false};
      std::atomic<std::uint32_t> waiter_id{0};
      std::atomic<bool> waiter_won{false};
      std::atomic<bool> barger_stole_it{false};

      thread owner_thread(
         [&]{
            // Must win the first CAS deterministically; see the note in the
            // single-core test above for why this gate is required.
            ASSERT_TRUE(r.try_acquire());
            owner_ready.store(true, std::memory_order_release);
            // Release only once waiter_armed proves the waiter has genuinely
            // parked (see waiter_block_witness below) AND the barger is
            // actively contending, so the release-time race is
            // deterministically live rather than dependent on incidental
            // timing. Because waiter_armed is proof, not a hint, release()
            // is GUARANTEED to find a non-empty queue and take the
            // commit-to-waiter branch, never the free branch, which is what
            // makes barging structurally impossible here rather than merely
            // unlikely: if barger_stole_it is ever observed true after this
            // gate, it is a genuine bug in wake_one_and_transfer, not a test
            // timing artifact.
            while (!barger_spinning.load(std::memory_order_acquire) ||
                   !waiter_armed.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            r.release();
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      thread waiter(
         [&]{
            while (!owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            waiter_id.store(this_thread::id(), std::memory_order_release);
            this_thread::wait_on(r);
            waiter_won.store(r.owner.load(std::memory_order_acquire) == this_thread::id(),
                             std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core1
      );

      // Pinned to the SAME core as waiter, at a strictly LOWER priority; see
      // the identical technique and rationale in the single-core test above.
      thread waiter_block_witness(
         [&]{ waiter_armed.store(true, std::memory_order_release); },
         witness_stack,
         thread::priority(1),
         core1
      );

      // The barger never parks: it spins on try_acquire, racing the release
      // from a core that did no waiting at all. This is exactly the shape
      // that would win against a plain wake_one-based release. It is gated
      // on owner_ready for the same deterministic-first-CAS reason as
      // owner_thread and waiter.
      thread barger(
         [&]{
            while (!owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            barger_spinning.store(true, std::memory_order_release);
            for (int i = 0; i < 100'000; ++i) {
               if (r.try_acquire()) {
                  barger_stole_it.store(true, std::memory_order_release);
                  return;
               }
               cyros_port_cpu_relax();
            }
         },
         barger_stack,
         thread::priority(0),
         core2
      );

      kernel::start();

      ASSERT_NE(waiter_id.load(), 0u);
      EXPECT_TRUE(waiter_won.load()) << "parked waiter did not receive ownership";
      EXPECT_FALSE(barger_stole_it.load()) << "a non-parked thread barged the transfer";
      kernel::finalise();
   }
}

/* ============================================================================
 * Priority ordering of the transfer, cross-core
 *
 * wake_one_and_transfer hands ownership to the SAME waiter arm() already selected
 * (best priority, FIFO within priority), it does not re-derive fairness. This
 * proves that property survives when the two parked waiters are parked from
 * different cores: a low-priority waiter parks first on core1, then a
 * higher-priority waiter parks on core2. Release must still favour the
 * higher-priority thread, matching arm()'s priority-ordered insert.
 * ========================================================================= */
TEST_F(MultiCoreTransfer_Test,
       GivenTwoWaitersOnDifferentCoresAtDifferentPriority_WhenReleased_ThenHigherPriorityWaiterWinsTransfer)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack owner_stack;
      cyros::test::guarded_stack low_stack;
      cyros::test::guarded_stack low_witness_stack;
      cyros::test::guarded_stack high_stack;
      cyros::test::guarded_stack high_witness_stack;

      TransferResource r;
      std::atomic<bool> owner_ready{false};
      std::atomic<std::uint32_t> low_id{0};
      std::atomic<std::uint32_t> high_id{0};
      std::atomic<bool> low_parked{false};
      std::atomic<bool> low_armed{false};
      std::atomic<bool> high_armed{false};
      std::atomic<std::uint32_t> transfer_winner{0};

      thread owner_thread(
         [&]{
            // Must win the first CAS deterministically; see the note in the
            // single-core test above for why this gate is required.
            ASSERT_TRUE(r.try_acquire());
            owner_ready.store(true, std::memory_order_release);
            // Release only once BOTH block-witnesses below prove their
            // sibling has genuinely armed. Gating on a "parked" flag set
            // before wait_on() is not enough, since arm happens inside
            // wait_on: releasing before both are truly armed could free the
            // resource into an empty-queue race between low and high
            // (whoever wins a raw CAS keeps it, with nobody left to release
            // it again), which is not the priority-ordering property this
            // test exists to check, and would strand the loser forever.
            while (!low_armed.load(std::memory_order_acquire) ||
                   !high_armed.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            r.release();
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      // Lower numeric priority value assumed higher urgency, matching the
      // rest of the suite's priority(0) == top convention; low waiter is
      // intentionally the LOWER-urgency thread (priority 2).
      thread low_waiter(
         [&]{
            while (!owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            low_id.store(this_thread::id(), std::memory_order_release);
            // low_parked here sequences ARRIVAL ORDER only (low attempts to
            // arm before high), so the test proves arm()'s priority-ordered
            // insert beats arrival order, not merely that it happens to
            // agree with it. It is NOT what gates owner_thread's release;
            // low_armed/high_armed (via the witnesses below) do that.
            low_parked.store(true, std::memory_order_release);
            this_thread::wait_on(r);
            if (r.owner.load(std::memory_order_acquire) == this_thread::id()) {
               // Record-once: owner_thread releases exactly ONCE, so only
               // one of low/high can be the genuine, direct recipient. If
               // the other later ends up owning r too (via the hand-back
               // below), it must not overwrite the real winner.
               std::uint32_t expected = 0;
               transfer_winner.compare_exchange_strong(expected, this_thread::id(),
                                                        std::memory_order_acq_rel);
               // Hand back what we hold. Without this, whichever of low/high
               // is NOT the priority-correct winner remains permanently
               // parked, since owner_thread never releases a second time:
               // kernel::start() would then never reach quiescence. If the
               // sibling is still parked this transfers to it (which is how
               // the loser above ever gets woken at all); otherwise it is a
               // harmless free.
               r.release();
            }
         },
         low_stack,
         thread::priority(2),
         core1
      );

      // Pinned to the SAME core as low_waiter, at a strictly LOWER priority;
      // see the technique and rationale in the single-core test above.
      thread low_witness(
         [&]{ low_armed.store(true, std::memory_order_release); },
         low_witness_stack,
         thread::priority(3),
         core1
      );

      thread high_waiter(
         [&]{
            // Attempt to arm after the low-priority waiter, so a correct
            // implementation must place high ahead of low by PRIORITY
            // despite arriving second, ruling out an implementation that is
            // secretly FIFO-ordered rather than priority-ordered.
            while (!low_parked.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            high_id.store(this_thread::id(), std::memory_order_release);
            this_thread::wait_on(r);
            if (r.owner.load(std::memory_order_acquire) == this_thread::id()) {
               // Same record-once + hand-back pattern as low_waiter above.
               std::uint32_t expected = 0;
               transfer_winner.compare_exchange_strong(expected, this_thread::id(),
                                                        std::memory_order_acq_rel);
               r.release();
            }
         },
         high_stack,
         thread::priority(1),
         core2
      );

      // Pinned to the SAME core as high_waiter, at a strictly LOWER
      // priority; same technique, distinct core from low_witness so it
      // tracks high_waiter specifically.
      thread high_witness(
         [&]{ high_armed.store(true, std::memory_order_release); },
         high_witness_stack,
         thread::priority(2),
         core2
      );

      kernel::start();

      ASSERT_NE(low_id.load(), 0u);
      ASSERT_NE(high_id.load(), 0u);
      EXPECT_EQ(transfer_winner.load(), high_id.load())
         << "wake_one_and_transfer did not honour arm()'s priority ordering across cores";
      kernel::finalise();
   }
}

/* ============================================================================
 * Empty-queue release under cross-core race (lost-wakeup guard)
 *
 * The property that makes wake_one_and_transfer's empty-case commit have to happen
 * under the SAME lock as the pop: a thread on another core may be mid-arm
 * (about to poll try_satisfy and park) at the exact moment release() finds
 * the queue empty. If the empty-case commit (owner = 0) happened outside the
 * lock, the arming thread could poll-see "still owned", commit to park, and
 * then the late owner=0 store would leave it parked forever with no future
 * wake to recover it (the resource is free but nothing will ever release it
 * again in this test, since no third party will call release()).
 *
 * This is run under stress specifically to hunt that timing window: the
 * would-be-waiter's wait_on() races the owner's release() from another core,
 * repeatedly, so if the race exists it should eventually surface as a hang
 * (caught by the test framework's own timeout) rather than a graceful
 * failure.
 * ========================================================================= */
TEST_F(MultiCoreTransfer_Test,
       GivenRacingArmAndEmptyQueueRelease_WhenBothLandOnDifferentCores_ThenNoLostWakeup)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack owner_stack;
      cyros::test::guarded_stack racer_stack;

      TransferResource r;
      std::atomic<bool> racer_completed{false};

      thread owner_thread(
         [&]{
            // Unlike every other test in this suite, deliberately NO
            // owner_ready gate here: this test's whole point is the
            // unsynchronised race, so racer may legitimately win the
            // initial CAS via try_satisfy's uncontended-take branch before
            // this thread ever runs. That is a valid, expected outcome (see
            // racer's comment below), not a bug, so this must not assert
            // success. Only release what was actually acquired: if racer
            // already took r directly, there is nothing here to give back,
            // and racer completes without ever parking.
            if (r.try_acquire()) {
               // Release immediately with (as far as this core knows) an
               // empty queue: no synchronisation with the racer beyond the
               // natural scheduling jitter across cores, which is exactly
               // the window under test.
               r.release();
            }
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      thread racer(
         [&]{
            // May observe the resource as already free (take it directly via
            // try_satisfy's CAS branch) or as still owned (park, then be
            // handed ownership if release's pop-and-commit hasn't happened
            // yet, or free it itself if the CAS branch fires on a later
            // poll). Either outcome is correct; only NOT COMPLETING is a bug.
            this_thread::wait_on(r);
            racer_completed.store(true, std::memory_order_release);
         },
         racer_stack,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(racer_completed.load())
         << "racer never completed - suspect a lost wakeup in the empty-queue release path";
      kernel::finalise();
   }
}

/* ============================================================================
 * wait_on_any composition: wake_one_and_transfer-released resource alongside a plain
 * barge-permitting waitable
 *
 * Proves wake_one_and_transfer's resource stays a first-class waitable: a thread can
 * block on {TransferResource, ordinary TestWaitable-style source} in one
 * wait_on_any call, and a wake_one_and_transfer release still resolves it correctly
 * (winner index for the transfer resource) even though the OTHER source in
 * the group is woken with plain wake_one semantics. This is the composition
 * the design is meant to preserve, unlike a separate substrate type
 * which could not participate in wait_on_any at all.
 * ========================================================================= */
TEST_F(MultiCoreTransfer_Test,
       GivenWaiterBlockedOnTransferResourceAndPlainWaitable_WhenTransferReleases_ThenCorrectIndexWins)
{
   for (int rep = 0; rep < stress_repetitions; ++rep) {
      SCOPED_TRACE("stress rep " + std::to_string(rep));

      kernel::initialise();

      cyros::test::guarded_stack owner_stack;
      cyros::test::guarded_stack waiter_stack;
      cyros::test::guarded_stack witness_stack;

      class PlainWaitable final : public waitable
      {
      public:
         std::atomic<bool> condition{false};
      protected:
         bool try_satisfy() noexcept override
         {
            return condition.load(std::memory_order_acquire);
         }
      };

      TransferResource r;
      PlainWaitable never_fires; // stays unsatisfied for the test's duration
      std::atomic<bool> owner_ready{false};
      std::atomic<bool> waiter_armed{false};
      std::atomic<std::uint32_t> waiter_id{0};
      std::atomic<std::size_t> winner{static_cast<std::size_t>(-1)};

      thread owner_thread(
         [&]{
            // Must win the first CAS deterministically; see the note in the
            // single-core test above for why this gate is required.
            ASSERT_TRUE(r.try_acquire());
            owner_ready.store(true, std::memory_order_release);
            // Release only once waiter_armed proves the waiter has genuinely
            // armed via wait_on_any (which arms through the same
            // wait_queue::arm() as wait_on), not merely signalled intent to.
            while (!waiter_armed.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            r.release();
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      thread waiter(
         [&]{
            while (!owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            waiter_id.store(this_thread::id(), std::memory_order_release);
            winner.store(this_thread::wait_on_any(r, never_fires), std::memory_order_release);
         },
         waiter_stack,
         thread::priority(0),
         core1
      );

      // Pinned to the SAME core as waiter, at a strictly LOWER priority; see
      // the technique and rationale in the single-core test above.
      thread waiter_block_witness(
         [&]{ waiter_armed.store(true, std::memory_order_release); },
         witness_stack,
         thread::priority(1),
         core1
      );

      kernel::start();

      ASSERT_NE(waiter_id.load(), 0u);
      EXPECT_EQ(winner.load(), 0U) << "transfer resource did not win its wait_on_any slot";
      EXPECT_EQ(r.owner.load(), waiter_id.load());
      kernel::finalise();
   }
}