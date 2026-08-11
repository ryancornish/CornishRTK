#include <cyros/sync/mutex.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

using namespace cyros;
static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* ============================================================================
 * Priority inheritance, black-box
 *
 * Everything here observes PI through the public surface only: lock, unlock,
 * try_lock, wait_on_any, and thread::get_priority, which reads effective
 * priority and is therefore the direct observation port for donation and
 * restore. No kernel internals are touched, so these tests survive internal
 * refactors (kernel_state consolidation included).
 *
 * Failure modes are engineered to be assertions, not hangs, wherever the
 * physics allows: every conductor poll has a bounded budget and a bail path
 * that releases all gates so the system can quiesce and the recorded failure
 * flag can be asserted in the test body. The one exception is a genuinely
 * broken transfer (a lost wakeup), which can only manifest as a hang, and the
 * framework timeout is the correct detector for that.
 *
 * Choreography discipline (preempt port, no slice timer): every spin gate
 * waits on a value set from a DIFFERENT core, every same-core dependency is a
 * real block, and each cross-thread stage is sequenced so no thread spins on
 * a peer that cannot run. Priorities are chosen so witnesses run exactly when
 * their subject has vacated the core.
 * ========================================================================= */

namespace
{

// Bounded budget for conductor polls that a PI failure would otherwise turn
// into an infinite spin. Generous by orders of magnitude against IPI plus
// reschedule latency, but sized remembering that debug builds do not inline
// cyros_port_cpu_relax, so every iteration is a real call and a failed stage
// should still report within a couple of seconds rather than minutes.
constexpr std::uint64_t poll_budget = 20'000'000;

// Extra spins the boosted holder burns after the exit signal before checking
// that the medium thread never started. Dwarfs the ready-request IPI latency,
// so the "medium was ready but could not preempt" claim is genuinely
// exercised rather than won by racing the intake.
constexpr int grace_spins = 200'000;

// Lifecycle repetitions for the deterministic donation tests. Each rep is a
// full kernel lifecycle, so kept modest, the properties are deterministic and
// the reps only add cold-start variety.
constexpr int pi_reps = 20;

// The renounce hunt runs many rounds INSIDE one lifecycle (cheap) times a few
// lifecycles (cold starts). Each round is one genuine dual-transfer race.
constexpr int renounce_lifecycles = 3;
constexpr int renounce_rounds     = 400;

// Bounded try_lock probe proving a renounced mutex is acquirable again. Only
// a renounce failure burns the whole budget.
constexpr int probe_budget = 10'000'000;

struct alignas(CYROS_PORT_STACK_ALIGN) aligned_stack
{
   std::array<std::byte, STACK_SIZE> bytes;
};

// Spin until predicate or budget exhaustion. Returns false on exhaustion so
// callers can record a failure and take their bail path.
template <typename Predicate>
[[nodiscard]] bool bounded_poll(Predicate&& done) noexcept
{
   for (std::uint64_t i = 0; i < poll_budget; ++i) {
      if (done()) return true;
      cyros_port_cpu_relax();
   }
   return false;
}

class SyncMutexPi_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * Inversion is bounded: the flagship
 *
 * The scenario PI exists for. On core0: H (prio 1), Mid (prio 3), L (prio 5).
 * L holds the mutex, H blocks on it, and Mid is a CPU-bound spinner that
 * under strict priority scheduling WITHOUT inheritance would preempt L (3
 * beats 5) the moment it becomes ready and starve the lock holder forever,
 * leaving H blocked behind a holder that never runs: unbounded inversion.
 * With inheritance, L runs at H's priority 1 for the whole critical section,
 * Mid (3) cannot preempt it, and the chain drains promptly.
 *
 * Observed three ways per rep:
 *   1. Donation: the conductor polls L's public priority to 1 after H parks.
 *   2. Prevention: Mid is made ready mid-CS (its gate is released and a grace
 *      window is burned) yet mid_started stays false until L exits the CS.
 *   3. Restore: H, which runs strictly after L's release recompute, reads L
 *      back at base priority 5.
 *
 * The gate mutexes double as choreography (conductor pre-locks them, releases
 * to schedule each actor) which also exercises transfer on a second and third
 * mutex instance per rep for free.
 * ========================================================================= */
TEST_F(SyncMutexPi_Test, GivenSpinnerBetweenHolderAndWaiter_WhenHighBlocksOnHeldMutex_ThenHolderInheritsAndSpinnerCannotStarveIt)
{
   for (int rep = 0; rep < pi_reps; ++rep) {
      SCOPED_TRACE("inversion rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         mutex m;
         mutex g_h;   // pre-locked by conductor, released to unleash H
         mutex g_mid; // pre-locked by conductor, released to unleash Mid

         thread* l{nullptr};

         std::atomic<bool> conductor_ready{false};
         std::atomic<bool> l_locked{false};
         std::atomic<bool> exit_cs{false};
         std::atomic<bool> mid_started{false};
         std::atomic<bool> mid_stop{false};
         std::atomic<bool> h_done{false};

         // Recorded observations, asserted in the test body.
         std::atomic<bool> boost_observed{false};
         std::atomic<bool> spinner_preempted_cs{false};
         std::atomic<std::uint8_t> l_priority_seen_by_h{0xFF};
         std::atomic<std::uint8_t> failed_stage{0}; // 0 means all stages passed
      } s;

      thread l(
         [&s]{
            // Runs only once H and Mid are parked on their gates (strict
            // priority, they outrank us), so the take is uncontended.
            s.m.lock();
            s.l_locked.store(true, std::memory_order_release);

            // Critical section: hold until the conductor has staged the
            // inversion (H parked and provably donating, Mid provably ready).
            // Cross-core flag, cannot wedge.
            while (!s.exit_cs.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }

            // Grace window: Mid's ready request has had orders of magnitude
            // longer than an IPI to land. If we are still running, priority
            // inheritance is what kept us on the core.
            for (int i = 0; i < grace_spins; ++i) {
               cyros_port_cpu_relax();
            }
            s.spinner_preempted_cs.store(s.mid_started.load(std::memory_order_acquire),
                                         std::memory_order_release);

            s.m.unlock(); // hands to H, restores us to base
         },
         stacks[0].bytes,
         thread::priority(5),
         core0
      );

      thread h(
         [&s]{
            while (!s.conductor_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.g_h.lock(); // parks until the conductor stages us
            s.g_h.unlock();

            s.m.lock(); // parks behind L, donating priority 1

            // We run strictly after L's release recompute on this core, so
            // this is a deterministic read of the restored base priority.
            s.l_priority_seen_by_h.store(s.l->get_priority(), std::memory_order_release);

            s.m.unlock();
            s.mid_stop.store(true, std::memory_order_release);
            s.h_done.store(true, std::memory_order_release);
         },
         stacks[1].bytes,
         thread::priority(1),
         core0
      );

      thread mid(
         [&s]{
            while (!s.conductor_ready.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            // Parked here until the conductor stages the inversion. Without
            // this gate Mid monopolises core0 from the first schedule and
            // starves L before it can even take the mutex, wedging the
            // choreography instead of testing the kernel.
            s.g_mid.lock();
            s.g_mid.unlock();

            // First observable action after release. If this flips while the
            // boosted L is inside its critical section, inheritance failed to
            // prevent the preemption and the CS check above records it.
            s.mid_started.store(true, std::memory_order_release);
            while (!s.mid_stop.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax(); // the starvation threat, defanged by PI
            }
         },
         stacks[2].bytes,
         thread::priority(3),
         core0
      );

      thread conductor(
         [&s]{
            // Stage 0: arm the gates BEFORE anyone can race for them.
            const bool gh = s.g_h.try_lock();
            const bool gm = s.g_mid.try_lock();
            CYROS_ASSERT(gh && gm);
            s.conductor_ready.store(true, std::memory_order_release);

            bool g_h_released   = false;
            bool g_mid_released = false;

            // Record the dead stage, then blow open everything still gated so
            // the actors unwind and the kernel quiesces for the assertions.
            // Order matters: mid_stop before g_mid so a released Mid exits
            // its spin immediately instead of starving L again.
            auto bail = [&](std::uint8_t stage) {
               s.failed_stage.store(stage, std::memory_order_relaxed);
               s.mid_stop.store(true, std::memory_order_release);
               s.exit_cs.store(true, std::memory_order_release);
               if (!g_h_released)   s.g_h.unlock();
               if (!g_mid_released) s.g_mid.unlock();
            };

            // Stage 1: the holder owns the mutex.
            if (!bounded_poll([&]{ return s.l_locked.load(std::memory_order_acquire); })) {
               return bail(1);
            }

            // Stage 2: unleash H, then prove donation landed via the public
            // priority. This poll IS the boost assertion.
            s.g_h.unlock();
            g_h_released = true;
            if (!bounded_poll([&]{ return s.l->get_priority() == 1; })) {
               return bail(2);
            }
            s.boost_observed.store(true, std::memory_order_release);

            // Stage 3: unleash Mid. On return the transfer is committed and
            // Mid's ready request is in flight to core0, where the boosted
            // holder must keep the core regardless.
            s.g_mid.unlock();
            g_mid_released = true;
            s.exit_cs.store(true, std::memory_order_release);

            if (!bounded_poll([&]{ return s.h_done.load(std::memory_order_acquire); })) {
               return bail(3);
            }
         },
         stacks[3].bytes,
         thread::priority(0),
         core1
      );

      s.l = &l;

      kernel::start();

      EXPECT_EQ(s.failed_stage.load(), 0)
         << "inversion choreography failed at stage " << int(s.failed_stage.load())
         << " (1: holder never locked, 2: donation never observed, 3: chain never drained)";
      EXPECT_TRUE(s.boost_observed.load())        << "holder never observed at donated priority 1";
      EXPECT_FALSE(s.spinner_preempted_cs.load()) << "medium spinner preempted the boosted holder inside its CS";
      EXPECT_EQ(s.l_priority_seen_by_h.load(), 5) << "holder not restored to base priority after unlock";

      kernel::finalise();
   }
}

/* ============================================================================
 * Transitive donation across cores, staged, with restores
 *
 * A(1, core2) blocks on M2 held by B(3, core1), which is blocked on M1 held
 * by C(5, core0). Donation must hop the chain: A boosts B, and because B is
 * blocked its re-slotted position on M1 changes that queue's best waiter,
 * which must chase through to C on yet another core.
 *
 * The conductor stages the chain and asserts each link through the public
 * priority, in order:
 *   C == 3   single hop landed (B parked on M1)
 *   C == 1   transitive hop landed (A parked on M2, chased B -> C)
 *   B == 1   the mid-chain thread carries the donation too
 *   C == 5   restore after C releases M1
 *   B == 1   B RETAINS A's donation while it runs holding M2
 *   B == 3   restore after B releases M2
 *
 * One thread per core, so every spin below waits on another core by
 * construction.
 * ========================================================================= */
TEST_F(SyncMutexPi_Test, GivenBlockedHolderChainAcrossCores_WhenUrgentWaiterJoins_ThenDonationChasesTransitivelyAndRestoresStepwise)
{
   for (int rep = 0; rep < pi_reps; ++rep) {
      SCOPED_TRACE("chain rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         mutex m1; // held by C
         mutex m2; // held by B

         thread* b{nullptr};
         thread* c{nullptr};

         std::atomic<bool> c_locked{false};
         std::atomic<bool> b_locked{false};
         std::atomic<bool> b_got_m1{false};
         std::atomic<bool> a_go{false};
         std::atomic<bool> a_got_m2{false};
         std::atomic<bool> release_c{false};
         std::atomic<bool> release_b{false};
         std::atomic<bool> a_exit{false};

         std::atomic<std::uint8_t> failed_stage{0}; // 0 means all stages passed
      } s;

      thread c_thread(
         [&s]{
            s.m1.lock();
            s.c_locked.store(true, std::memory_order_release);
            while (!s.release_c.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m1.unlock(); // transfer to B, restore to base 5
         },
         stacks[0].bytes,
         thread::priority(5),
         core0
      );

      thread b_thread(
         [&s]{
            while (!s.c_locked.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m2.lock();
            s.b_locked.store(true, std::memory_order_release);
            s.m1.lock(); // parks behind C, donating 3
            s.b_got_m1.store(true, std::memory_order_release);
            while (!s.release_b.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m1.unlock(); // frees, no waiters remain on M1
            s.m2.unlock(); // transfer to A, restore to base 3
         },
         stacks[1].bytes,
         thread::priority(3),
         core1
      );

      thread a_thread(
         [&s]{
            while (!s.a_go.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m2.lock(); // parks behind B: the transitive trigger
            s.a_got_m2.store(true, std::memory_order_release);
            while (!s.a_exit.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.m2.unlock();
         },
         stacks[2].bytes,
         thread::priority(1),
         core2
      );

      thread conductor(
         [&s]{
            // Bail: record which stage died, then blow every gate open so the
            // chain unwinds and the kernel can quiesce for the assertion.
            auto bail = [&s](std::uint8_t stage) {
               s.failed_stage.store(stage, std::memory_order_relaxed);
               s.a_go.store(true, std::memory_order_release);
               s.release_c.store(true, std::memory_order_release);
               s.release_b.store(true, std::memory_order_release);
               s.a_exit.store(true, std::memory_order_release);
            };

            if (!bounded_poll([&]{ return s.b_got_m1.load(std::memory_order_acquire)
                                       || s.b_locked.load(std::memory_order_acquire); })) {
               return bail(1); // setup never formed
            }

            // Single hop: B parked on M1 must lift C to 3.
            if (!bounded_poll([&]{ return s.c->get_priority() == 3; })) {
               return bail(2);
            }

            // Transitive hop: unleash A, the donation must chase to C.
            s.a_go.store(true, std::memory_order_release);
            if (!bounded_poll([&]{ return s.c->get_priority() == 1; })) {
               return bail(3);
            }
            if (!bounded_poll([&]{ return s.b->get_priority() == 1; })) {
               return bail(4); // mid-chain thread missing its donation
            }

            // Unwind link one: C releases, must restore to base.
            s.release_c.store(true, std::memory_order_release);
            if (!bounded_poll([&]{ return s.c->get_priority() == 5; })) {
               return bail(5);
            }

            // B now runs holding M2 with A parked on it: B must RETAIN A's
            // donation, this is restore correctness (per-resource truth, not
            // a blanket reset).
            if (!bounded_poll([&]{ return s.b_got_m1.load(std::memory_order_acquire); })) {
               return bail(6);
            }
            if (s.b->get_priority() != 1) {
               return bail(7);
            }

            // Unwind link two.
            s.release_b.store(true, std::memory_order_release);
            if (!bounded_poll([&]{ return s.b->get_priority() == 3; })) {
               return bail(8);
            }
            if (!bounded_poll([&]{ return s.a_got_m2.load(std::memory_order_acquire); })) {
               return bail(9);
            }
            s.a_exit.store(true, std::memory_order_release);
         },
         stacks[3].bytes,
         thread::priority(0),
         core3
      );

      s.b = &b_thread;
      s.c = &c_thread;

      kernel::start();

      EXPECT_EQ(s.failed_stage.load(), 0)
         << "transitive PI chain failed at stage " << int(s.failed_stage.load());

      kernel::finalise();
   }
}

