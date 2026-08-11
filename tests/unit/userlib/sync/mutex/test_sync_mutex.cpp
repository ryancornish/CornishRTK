#include <cyros/sync/mutex.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include "gtest/gtest.h"

#include <type_traits>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;

/* A mutex must never be composable into a wait_on_any group, and the ban is
 * enforced by the type system rather than by a runtime check: sync::mutex is a
 * facade over base_mutex, which is deliberately NOT a waitable.
 *
 * This assertion is what two deleted tests turned into. The renounce-window
 * race was only constructible through that composition, so making it
 * ill-formed retires that machinery and its test together. A compile-time
 * check is the right shape for that half: there is no runtime behaviour left
 * to observe.
 *
 * The already-admitted handover hole (cross-core-defects.md 8.1) is NOT
 * retired by the ban, only its deterministic test was. It has a second
 * constructor that needs no group wait: a thread armed on a mutex while
 * holding nothing can be preempted between arming and parking and rotated
 * into the ready matrix, and a handover landing at that moment reaches an
 * already-admitted owner. The re-route in scheduler::set_thread_ready is what
 * migrates such an owner to the holder list, and it stays load-bearing.
 * See mutex-first-class-plan.md D6. */
static_assert(!std::is_convertible_v<mutex&, waitable&>,
              "a mutex must not be usable as a waitable, or it can enter a wait_on_any group");
static_assert(!std::is_base_of_v<waitable, mutex>,
              "a mutex must not derive from waitable");

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* ============================================================================
 * What this suite tests, and how it observes it
 *
 * sync::mutex is the productionised form of the transfer resource proven in
 * test_transfer_waitables: try_lock() is a non-blocking CAS take, lock() parks
 * on the mutex via wait_on, and unlock() is wake_one_and_transfer, which is
 * what makes the mutex barge-free (ownership is committed to the woken waiter
 * under the queue lock, so no fresh try_lock can interpose). The transfer test
 * already stresses that mechanism at the waitable level, so this suite is the
 * lighter, mutex-level wiring check: it drives ONLY the public surface
 * (try_lock / lock / unlock) and never reads the private owner field. Every
 * property below is inferred from observable behaviour instead:
 *   - try_lock()'s bool result,
 *   - an atomic in-critical-section occupancy sentinel,
 *   - the ORDER in which threads return from lock().
 *
 * The timed acquire paths (try_lock_for / try_lock_until) are intentionally
 * NOT exercised: they are not linked in this build.
 *
 * Shared state lives in a per-test struct captured by a single reference. That
 * keeps each thread entry closure to one pointer (thread::entry_fn has a fixed
 * 48-byte inline buffer and hard-errors on overflow) and names every field the
 * threads coordinate through.
 *
 * Latency note
 * ------------
 * On the preempt backend the dominant cost is the number of full kernel
 * lifecycles: every kernel::start() at cores == 4 spawns and joins three
 * per-core pthreads and installs their signal interceptors. So the heavy
 * mutual-exclusion stress below runs many thousands of contended rounds
 * INSIDE a handful of lifecycles (the round_robin model) rather than one race
 * per lifecycle repeated hundreds of times. The race-sensitive handoff tests
 * do need a fresh setup per window, so they keep a MODEST rep count rather
 * than the transfer suite's 200.
 *
 * Preempt-without-a-timer discipline
 * ----------------------------------
 * This suite starts no slice timer, so two equal-priority threads on the same
 * core never rotate. Consequently no thread here busy-spins on a flag that
 * only a same-core equal-priority peer can set (that would wedge). Every spin
 * gate waits on a flag set from a DIFFERENT core (resolves in microseconds on
 * genuinely parallel pthreads), and every same-core dependency is expressed as
 * a real lock() block, which voluntarily yields the core. The witness threads
 * that detect "a waiter has parked" are strictly lower priority on the
 * waiter's own core, so strict priority scheduling only lets them run once the
 * waiter has actually blocked in lock().
 * ========================================================================= */

namespace
{

// Cold-start variety for the contention test: how many times we tear the
// kernel down and back up. Kept small because each one pays the 4-core
// pthread bring-up tax. The RACE coverage comes from rounds below, not this.
constexpr int contention_lifecycles = 4;

// Contended acquisitions each worker performs per lifecycle. This is the real
// intensity knob: bump it to hunt mutual-exclusion races harder, it costs
// almost nothing because it stays inside a single already-paid-for lifecycle.
constexpr std::uint64_t contention_rounds = 5000;

// One worker per core so all contention is genuine cross-core simultaneous
// acquisition. More than one equal-priority worker per core would starve
// under the no-timer scheduling above until the running one happened to block.
constexpr std::size_t contention_workers = 4;

// Widens the critical section by a few relaxes so that a hypothetical
// mutual-exclusion violation actually OVERLAPS observably in the occupancy
// sentinel, rather than being a vanishingly short two-owner window that the
// sentinel might step over. Cheap, and only affects detectability.
constexpr int critical_section_widen = 8;

// Per-lifecycle reps for the handoff tests. These genuinely need a fresh
// parked-waiter setup each time, so each rep is a full lifecycle. Modest on
// purpose: the barge-free and priority mechanisms themselves are already
// stress-proven in test_transfer_waitables, here we only confirm the mutex
// wires them through correctly.
constexpr int handoff_reps = 50;

struct alignas(CYROS_PORT_STACK_ALIGN) aligned_stack
{
   std::array<std::byte, STACK_SIZE> bytes;
};

// Each test owns its own kernel lifecycle, so the fixture is empty. Inheriting
// from ::testing::Test (rather than bare TEST) keeps naming consistent with
// the other kernel/userlib suites.
class SyncMutex_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * Uncontended contract, single thread
 *
 * The baseline nobody should be able to break: a free mutex is takeable by
 * try_lock, a mutex already held BY THE CALLER rejects a second try_lock (it
 * is not recursive), unlock frees it, and lock/unlock round-trips without
 * blocking when there is no contender. All on one thread on one core, so there
 * is no timing at all here, just the state machine.
 * ========================================================================= */
TEST_F(SyncMutex_Test, GivenNoContention_WhenSingleThreadDrivesTryLockAndLock_ThenContractHolds)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> driver_stack{};

   struct state
   {
      mutex             m;
      std::atomic<bool> first_take{false};
      std::atomic<bool> recursive_rejected{false};
      std::atomic<bool> retake_after_unlock{false};
      std::atomic<bool> lock_unlock_returned{false};
   } s;

   thread driver(
      [&s]{
         // Free mutex: first try_lock must win.
         s.first_take.store(s.m.try_lock(), std::memory_order_release);

         // Held by us already: a second try_lock must FAIL. The mutex is not
         // recursive, so the owner re-attempting still sees a non-zero owner
         // and the CAS-from-zero fails. Record the negation so a true here
         // means "correctly rejected".
         s.recursive_rejected.store(!s.m.try_lock(), std::memory_order_release);

         s.m.unlock();

         // Now free again: try_lock must succeed a second time.
         s.retake_after_unlock.store(s.m.try_lock(), std::memory_order_release);
         s.m.unlock();

         // The blocking API on a free mutex must simply acquire and return
         // without parking. If lock() ever failed to return here the kernel
         // would never quiesce and the test would hang, so reaching the store
         // is itself the proof.
         s.m.lock();
         s.m.unlock();
         s.lock_unlock_returned.store(true, std::memory_order_release);
      },
      driver_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   EXPECT_TRUE(s.first_take.load())           << "try_lock failed on a free mutex";
   EXPECT_TRUE(s.recursive_rejected.load())   << "try_lock succeeded recursively (mutex is not recursive)";
   EXPECT_TRUE(s.retake_after_unlock.load())  << "try_lock failed after unlock freed the mutex";
   EXPECT_TRUE(s.lock_unlock_returned.load()) << "lock()/unlock() did not round-trip on a free mutex";

   kernel::finalise();
}

/* ============================================================================
 * try_lock rejection while another core holds the mutex
 *
 * The cross-core half of try_lock's contract: whilst a holder on one core is
 * inside its critical section, a try_lock from a DIFFERENT core must fail (it
 * must not barge in on a live owner). Observed purely via the returned bool.
 *
 * The holder keeps the mutex until the observer has finished its attempt, so
 * there is no window in which the mutex is legitimately free during the probe.
 * Both flags are cross-core, so neither spin can wedge under the no-timer
 * scheduling.
 * ========================================================================= */
TEST_F(SyncMutex_Test, GivenHeldByAnotherCore_WhenTryLockProbes_ThenItFails)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> holder_stack{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> observer_stack{};

   struct state
   {
      mutex             m;
      std::atomic<bool> holder_owns{false};
      std::atomic<bool> observer_done{false};
      std::atomic<bool> probe_result{true};  // seeded true so a pass REQUIRES an observed false
   } s;

   thread holder(
      [&s]{
         // try_lock (not lock) so the holder deterministically owns the mutex
         // before it announces readiness. Nothing else touches the mutex until
         // holder_owns is seen, so this cannot be lost to a racing acquirer.
         const bool took = s.m.try_lock();
         ASSERT_TRUE(took);  // seeding the contention must not itself fail
         s.holder_owns.store(true, std::memory_order_release);

         // Hold across the observer's whole probe so the mutex is never
         // legitimately free while it is looking.
         while (!s.observer_done.load(std::memory_order_acquire)) {
            cyros_port_cpu_relax();
         }
         s.m.unlock();
      },
      holder_stack,
      thread::priority(0),
      core0
   );

   thread observer(
      [&s]{
         while (!s.holder_owns.load(std::memory_order_acquire)) {
            cyros_port_cpu_relax();
         }
         // Must fail: the mutex is held by the holder on core0.
         s.probe_result.store(s.m.try_lock(), std::memory_order_release);
         s.observer_done.store(true, std::memory_order_release);

         // Defensive: if the probe wrongly SUCCEEDED we now own the mutex and
         // must release it, otherwise the holder's unlock would race an owner
         // that is really us. On a correct mutex this branch never runs.
         if (s.probe_result.load(std::memory_order_acquire)) {
            s.m.unlock();
         }
      },
      observer_stack,
      thread::priority(0),
      core1
   );

   kernel::start();

   EXPECT_FALSE(s.probe_result.load()) << "try_lock barged a mutex held by another core";

   kernel::finalise();
}

/* ============================================================================
 * Mutual exclusion under cross-core contention (the flagship)
 *
 * The property the whole primitive exists for: no two threads are ever inside
 * the critical section at once. Four workers, one pinned per core, each loop
 * lock / critical-section / unlock for many rounds, all fighting over a single
 * mutex. Detection is twofold:
 *
 *   1. occupancy sentinel (well-defined): on entry a worker fetch_adds an
 *      atomic counter and asserts the PRIOR value was 0. If it ever sees a
 *      non-zero prior, two owners were live simultaneously. This check is
 *      race-free regardless of whether the mutex is correct.
 *
 *   2. shared_counter (deliberately NON-atomic): incremented only inside the
 *      critical section. Under a correct mutex every access is serialised by
 *      the lock, so there is no data race and the final value must equal the
 *      total number of rounds. A broken mutex that admitted two owners would
 *      race this counter (undefined behaviour, yes, but that race IS the bug
 *      being hunted) and typically drop updates, surfacing as a short count.
 *
 * All the intensity lives in contention_rounds, INSIDE each lifecycle, so this
 * stays cheap on the preempt backend no matter how hard we crank it. A worker
 * that cannot get the lock parks (a real yield), so there is no same-core spin
 * to wedge, and unlock's transfer wakes a parked peer on another core via the
 * IPI path.
 * ========================================================================= */
TEST_F(SyncMutex_Test, GivenFourCoresHammeringOneMutex_WhenTheyLockRepeatedly_ThenNeverTwoOwnersAndNoLostUpdates)
{
   static_assert(contention_workers <= config::cores, "one worker per core, no same-core equal-priority starvation");

   for (int life = 0; life < contention_lifecycles; ++life) {
      SCOPED_TRACE("contention lifecycle " + std::to_string(life));

      kernel::initialise();

      static std::array<aligned_stack, contention_workers> worker_stacks{};

      struct state
      {
         mutex            m;
         std::atomic<int> occupancy{0};
         std::atomic<bool> exclusion_violated{false};
         std::uint64_t     shared_counter{0};  // guarded solely by m, intentionally non-atomic
      } s;

      auto worker_body = [&s]{
         for (std::uint64_t r = 0; r < contention_rounds; ++r) {
            s.m.lock();

            // --- critical section -------------------------------------------
            const int prior = s.occupancy.fetch_add(1, std::memory_order_acq_rel);
            if (prior != 0) {
               // Someone else was already inside: mutual exclusion is broken.
               s.exclusion_violated.store(true, std::memory_order_relaxed);
            }

            ++s.shared_counter;  // safe only because the lock serialises us

            // Widen the window so a real violation overlaps observably.
            for (int w = 0; w < critical_section_widen; ++w) {
               cyros_port_cpu_relax();
            }

            s.occupancy.fetch_sub(1, std::memory_order_release);
            // --- end critical section ---------------------------------------

            s.m.unlock();
         }
      };

      std::array<thread, contention_workers> workers{};
      for (std::size_t i = 0; i < contention_workers; ++i) {
         workers[i] = thread(
            worker_body,
            worker_stacks[i].bytes,
            thread::priority(0),
            core_affinity::from_id(static_cast<std::uint32_t>(i))
         );
      }

      kernel::start();

      EXPECT_FALSE(s.exclusion_violated.load())
         << "two threads were inside the critical section at once";
      EXPECT_EQ(s.shared_counter, contention_rounds * contention_workers)
         << "lost or torn updates: a broken mutex let critical sections overlap";
      EXPECT_EQ(s.occupancy.load(), 0)
         << "occupancy did not return to zero, a critical section leaked";

      kernel::finalise();
   }
}

/* ============================================================================
 * Barge-free transfer, black-box
 *
 * The property wake_one_and_transfer gives the mutex: while a waiter is parked
 * in lock(), a FRESH thread on another core that never parked must not be able
 * to try_lock the mutex out from under the parked waiter when the owner
 * unlocks. The parked waiter must acquire FIRST.
 *
 * Observed without reading owner: each successful acquirer stamps a global
 * sequence number via fetch_add. A correct, barge-free unlock hands directly
 * to the parked waiter, so the waiter's stamp is 0. If the mutex barged, the
 * spinning try_lock thief would grab it first (stamp 0) and the parked waiter
 * would only get it later (stamp 1), failing the assertion.
 *
 * The owner takes the mutex deterministically first, then releases only once
 * the waiter is provably parked (waiter_armed, set by the same-core witness)
 * AND the barger is actively spinning, so the release-time race is genuinely
 * live every rep rather than by luck.
 * ========================================================================= */
TEST_F(SyncMutex_Test, GivenWaiterParkedAndFreshCoreProbing_WhenOwnerUnlocks_ThenParkedWaiterAcquiresFirst)
{
   for (int rep = 0; rep < handoff_reps; ++rep) {
      SCOPED_TRACE("handoff rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> owner_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> waiter_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> witness_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> barger_stack{};

      struct state
      {
         mutex                      m;
         std::atomic<bool>          owner_ready{false};
         std::atomic<bool>          waiter_armed{false};
         std::atomic<bool>          barger_spinning{false};
         std::atomic<bool>          waiter_acquired{false};
         std::atomic<std::uint32_t> acquire_seq{0};
         std::atomic<std::int32_t>  waiter_order{-1};
         std::atomic<std::int32_t>  barger_order{-1};
      } s;

      thread owner(
         [&s]{
            // Deterministic first take. Every other actor gates on owner_ready,
            // so nothing can win the mutex before this does.
            const bool took = s.m.try_lock();
            ASSERT_TRUE(took);
            s.owner_ready.store(true, std::memory_order_release);

            // Release only once the waiter has genuinely parked and the barger
            // is genuinely contending, so unlock is guaranteed to find a parked
            // waiter and take the transfer branch, not a free-and-race branch.
            // Both flags are set from OTHER cores, so this spin cannot wedge.
            while (!s.waiter_armed.load(std::memory_order_acquire) ||
                   !s.barger_spinning.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.unlock();
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      thread waiter(
         [&s]{
            while (!s.owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();  // parks, then must be handed ownership by owner's unlock
            s.waiter_acquired.store(true, std::memory_order_release);
            s.waiter_order.store(static_cast<std::int32_t>(s.acquire_seq.fetch_add(1, std::memory_order_acq_rel)),
                                 std::memory_order_release);
            // Free it so the system can quiesce. The barger is not parked, so
            // this frees the mutex rather than transferring.
            s.m.unlock();
         },
         waiter_stack,
         thread::priority(0),
         core1
      );

      // Same core as waiter, strictly lower priority. Under strict priority
      // scheduling with no slice timer, this can only run once the waiter has
      // vacated the core by BLOCKING inside lock(), so its execution is proof
      // the waiter has armed and parked. One relaxed store, no lock touched.
      thread witness(
         [&s]{ s.waiter_armed.store(true, std::memory_order_release); },
         witness_stack,
         thread::priority(1),
         core1
      );

      thread barger(
         [&s]{
            while (!s.owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.barger_spinning.store(true, std::memory_order_release);
            // Hammer try_lock against the owner's unlock from a core that did
            // no waiting. On a barge-free mutex this fails for the whole window
            // the waiter holds, so it simply gives up once the waiter has the
            // mutex. On a barging mutex it would steal ownership here and stamp
            // order 0. No fixed cap is needed: waiter_acquired is guaranteed to
            // become true (a lost wakeup instead would hang and be caught by
            // the framework timeout, which is the correct signal for that bug).
            while (!s.waiter_acquired.load(std::memory_order_acquire)) {
               if (s.m.try_lock()) {
                  s.barger_order.store(static_cast<std::int32_t>(s.acquire_seq.fetch_add(1, std::memory_order_acq_rel)),
                                       std::memory_order_release);
                  s.m.unlock();
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

      EXPECT_TRUE(s.waiter_acquired.load()) << "parked waiter never acquired the mutex";
      EXPECT_EQ(s.waiter_order.load(), 0)
         << "a fresh try_lock barged ahead of the parked waiter on unlock";

      kernel::finalise();
   }
}

/* ============================================================================
 * Priority-ordered handoff, black-box
 *
 * wake_one_and_transfer hands to the SAME best-priority waiter the wait queue
 * already selected, so when two waiters of different priority are parked on a
 * held mutex, unlock must wake the higher-priority one first. Proven across
 * cores and without reading owner: the higher-priority waiter must stamp
 * sequence 0.
 *
 * The low-priority waiter is nudged to ARRIVE first (low_parked) so a correct
 * result proves priority ordering beats arrival order, not merely that they
 * happen to agree. The owner releases only once BOTH waiters are provably
 * parked (their witnesses have fired), so unlock is choosing between two
 * genuinely queued waiters. Each waiter hands the mutex on after acquiring so
 * the loser is eventually woken and the system quiesces.
 * ========================================================================= */
TEST_F(SyncMutex_Test, GivenTwoWaitersOfDifferentPriorityParked_WhenOwnerUnlocks_ThenHigherPriorityAcquiresFirst)
{
   for (int rep = 0; rep < handoff_reps; ++rep) {
      SCOPED_TRACE("handoff rep " + std::to_string(rep));

      kernel::initialise();

      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> owner_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> low_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> low_witness_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> high_stack{};
      alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> high_witness_stack{};

      struct state
      {
         mutex                      m;
         std::atomic<bool>          owner_ready{false};
         std::atomic<bool>          low_parked{false};
         std::atomic<bool>          low_armed{false};
         std::atomic<bool>          high_armed{false};
         std::atomic<std::uint32_t> acquire_seq{0};
         std::atomic<std::int32_t>  low_order{-1};
         std::atomic<std::int32_t>  high_order{-1};
      } s;

      thread owner(
         [&s]{
            const bool took = s.m.try_lock();
            ASSERT_TRUE(took);
            s.owner_ready.store(true, std::memory_order_release);

            // Both witnesses prove both waiters are parked before we choose.
            // Releasing earlier could free the mutex into a raw CAS race
            // between the two waiters, which is not the ordering property under
            // test. Cross-core flags, so no wedge.
            while (!s.low_armed.load(std::memory_order_acquire) ||
                   !s.high_armed.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.unlock();  // must hand to the higher-priority waiter
         },
         owner_stack,
         thread::priority(0),
         core0
      );

      // Lower urgency (higher numeric priority). Arrives first on purpose.
      thread low_waiter(
         [&s]{
            while (!s.owner_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            // Arrival marker only, gating high's start. It does NOT gate the
            // owner's release, low_armed/high_armed do that, so the test proves
            // the priority-ordered pick and not merely arrival order.
            s.low_parked.store(true, std::memory_order_release);
            s.m.lock();
            s.low_order.store(static_cast<std::int32_t>(s.acquire_seq.fetch_add(1, std::memory_order_acq_rel)),
                              std::memory_order_release);
            // Hand on: if high is somehow still parked this transfers to it,
            // otherwise it frees. Without this the loser would park forever and
            // the kernel would never quiesce.
            s.m.unlock();
         },
         low_stack,
         thread::priority(2),
         core1
      );

      thread low_witness(
         [&s]{ s.low_armed.store(true, std::memory_order_release); },
         low_witness_stack,
         thread::priority(3),
         core1
      );

      // Higher urgency (lower numeric priority). Arms after low so priority
      // must beat arrival order.
      thread high_waiter(
         [&s]{
            while (!s.low_parked.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m.lock();
            s.high_order.store(static_cast<std::int32_t>(s.acquire_seq.fetch_add(1, std::memory_order_acq_rel)),
                               std::memory_order_release);
            s.m.unlock();
         },
         high_stack,
         thread::priority(1),
         core2
      );

      thread high_witness(
         [&s]{ s.high_armed.store(true, std::memory_order_release); },
         high_witness_stack,
         thread::priority(2),
         core2
      );

      kernel::start();

      EXPECT_EQ(s.high_order.load(), 0)
         << "unlock did not hand to the higher-priority waiter first";
      EXPECT_EQ(s.low_order.load(), 1)
         << "lower-priority waiter did not acquire second";

      kernel::finalise();
   }
}