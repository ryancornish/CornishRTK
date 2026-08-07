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
#include <string>

using namespace cyros;

static_assert(config::cores >= 4, "Test suite is designed for (at least) quad-core configuration");

// A semaphore must stay constant-initialisable, so a static one lands in .bss
// with no startup code and no initialisation-order surface. This is the reason
// the park list is a named-but-unusable type rather than opaque storage: opaque
// storage would need placement-new in the constructor and break this.
constinit static sync::semaphore constant_initialised_semaphore{1};

static constexpr auto STACK_SIZE = thread::min_stack_size + (16 * 1024);

/* ============================================================================
 * Semaphore contract, black-box
 *
 * The semaphore is barge-permitting by design (built on plain wake, not
 * transfer), so this suite deliberately asserts only what that design
 * promises: token accounting is exact, acquire on a zero count BLOCKS until a
 * release supplies a token, releases wake enough waiters, and no token is
 * ever lost or double-granted. It deliberately does NOT assert grant order
 * between concurrent waiters, a woken waiter races fresh arrivals for the
 * token and may lose, which is the documented barging trade.
 *
 * NOTE: GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease and the
 * producer/consumer accounting test are the regression tests for the
 * unconditional-decrement acquire() bug: an acquire that never blocks wraps
 * the counter and both tests fail loudly (accounting) or via the recorded
 * order flag (blocking) rather than by hanging.
 * ========================================================================= */

namespace
{

constexpr int contract_reps = 3;

// Producer/consumer accounting volume per lifecycle. All inside one
// lifecycle, so cheap to raise.
constexpr std::uint64_t tokens_total = 20'000;

struct alignas(CYROS_PORT_STACK_ALIGN) aligned_stack
{
   std::array<std::byte, STACK_SIZE> bytes;
};

class SyncSemaphore_Test : public ::testing::Test {};

}  // namespace

/* ============================================================================
 * Counting contract, single thread
 *
 * Pure state machine, no timing: n tokens grant exactly n try_acquires, the
 * n+1th fails, release restores exactly what it adds, and acquire on a
 * positive count takes the fast path without blocking (reaching the flag is
 * the proof, a wrongly parked thread would prevent quiescence).
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenTokens_WhenSingleThreadDrivesTheCount_ThenAccountingIsExact)
{
   kernel::initialise();

   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, STACK_SIZE> driver_stack{};

   struct state
   {
      semaphore sem{3};
      std::atomic<int>  takes_granted{0};
      std::atomic<bool> fourth_rejected{false};
      std::atomic<bool> fast_acquire_returned{false};
      std::atomic<std::size_t> peek_after_release{0};
   } s;

   thread driver(
      [&s]{
         int granted = 0;
         for (int i = 0; i < 3; ++i) {
            if (s.sem.try_acquire()) ++granted;
         }
         s.takes_granted.store(granted, std::memory_order_release);
         s.fourth_rejected.store(!s.sem.try_acquire(), std::memory_order_release);

         s.sem.release(2);
         s.peek_after_release.store(s.sem.peek(), std::memory_order_release);

         // Positive count: acquire must take the fast path and return.
         s.sem.acquire();
         s.sem.acquire();
         s.fast_acquire_returned.store(true, std::memory_order_release);
      },
      driver_stack,
      thread::priority(0),
      core0
   );

   kernel::start();

   EXPECT_EQ(s.takes_granted.load(), 3)        << "3 tokens did not grant exactly 3 try_acquires";
   EXPECT_TRUE(s.fourth_rejected.load())       << "try_acquire succeeded on an empty semaphore";
   EXPECT_EQ(s.peek_after_release.load(), 2u)  << "release(2) on empty did not read back as 2";
   EXPECT_TRUE(s.fast_acquire_returned.load()) << "acquire on a positive count failed to return";

   kernel::finalise();
}

/* ============================================================================
 * acquire on zero blocks until release
 *
 * The property the current unconditional-decrement acquire() violates. The
 * consumer must NOT get past acquire() before the producer's release, which
 * is proven by ordering, not timing: the producer sets released_first before
 * releasing, and the consumer records whether that flag was up when acquire
 * returned. A non-blocking acquire returns immediately, long before the
 * producer's gate opens, and records the violation.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenZeroCount_WhenAcquire_ThenBlocksUntilRelease)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("blocking rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 3> stacks{};

      struct state
      {
         semaphore sem{0};
         std::atomic<bool> consumer_waiting{false};
         std::atomic<bool> released_first{false};
         std::atomic<bool> acquire_after_release{false};
      } s;

      thread consumer(
         [&s]{
            s.sem.acquire(); // must park: count is zero
            s.acquire_after_release.store(s.released_first.load(std::memory_order_acquire),
                                          std::memory_order_release);
         },
         stacks[0].bytes,
         thread::priority(1),
         core0
      );

      // Same core as the consumer, strictly lower priority: runs only once
      // the consumer has vacated the core by parking inside acquire, so its
      // store is the attestation the producer gates on. A broken,
      // non-blocking acquire lets the consumer terminate instead, which ALSO
      // lets this run, and the ordering flag then records the violation.
      thread witness(
         [&s]{ s.consumer_waiting.store(true, std::memory_order_release); },
         stacks[1].bytes,
         thread::priority(2),
         core0
      );

      thread producer(
         [&s]{
            while (!s.consumer_waiting.load(std::memory_order_acquire)) {
               cyros_port_cpu_relax();
            }
            s.released_first.store(true, std::memory_order_release);
            s.sem.release();
         },
         stacks[2].bytes,
         thread::priority(0),
         core1
      );

      kernel::start();

      EXPECT_TRUE(s.acquire_after_release.load())
         << "acquire returned before any token was released (non-blocking acquire bug)";

      kernel::finalise();
   }
}

/* ============================================================================
 * Producer/consumer accounting under real contention
 *
 * Two producers mint a known total of tokens in mixed burst sizes across two
 * cores, two consumers on two other cores acquire until the total is
 * consumed. Exactness is the assertion: consumed == minted, the count ends at
 * zero, and a final try_acquire fails. A lost wakeup hangs (framework timeout
 * is the detector), a lost or double-granted token breaks the arithmetic, and
 * the wrapped-counter acquire bug fails the final-count checks immediately.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenProducersAndConsumersAcrossCores_WhenTokensFlow_ThenNoneAreLostOrInvented)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("accounting rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 4> stacks{};

      struct state
      {
         semaphore sem{0};
         std::atomic<std::uint64_t> consumed{0};
         std::atomic<std::size_t>   final_peek{~std::size_t{0}};
         std::atomic<bool>          post_drain_take{true};
      } s;

      static_assert(tokens_total % 4 == 0, "split evenly across two producers and two consumers");

      auto producer_body = [](state& s, std::uint64_t budget) {
         std::uint64_t minted = 0;
         while (minted < budget) {
            // Mixed burst sizes exercise release(1) and release(n) paths.
            std::uint64_t const burst = ((minted / 3) % 3) + 1;
            std::uint64_t const n = (burst < budget - minted) ? burst : (budget - minted);
            s.sem.release(static_cast<std::size_t>(n));
            minted += n;
         }
      };

      auto consumer_body = [](state& s) {
         for (std::uint64_t i = 0; i < tokens_total / 2; ++i) {
            s.sem.acquire();
            s.consumed.fetch_add(1, std::memory_order_relaxed);
         }
      };

      thread p0([&s, &producer_body]{ producer_body(s, tokens_total / 2); },
                stacks[0].bytes, thread::priority(1), core0);
      thread p1([&s, &producer_body]{ producer_body(s, tokens_total / 2); },
                stacks[1].bytes, thread::priority(1), core1);

      thread c0(
         [&s, &consumer_body]{ consumer_body(s); },
         stacks[2].bytes, thread::priority(1), core2);
      thread c1(
         [&s, &consumer_body]{
            consumer_body(s);
            // Runs the epilogue in one consumer to avoid racing the checks:
            // by the time BOTH consumers have their full share, every token
            // is spoken for. Spin for the sibling's completion cross-core.
            while (s.consumed.load(std::memory_order_acquire) < tokens_total) {
               cyros_port_cpu_relax();
            }
            s.final_peek.store(s.sem.peek(), std::memory_order_release);
            s.post_drain_take.store(s.sem.try_acquire(), std::memory_order_release);
         },
         stacks[3].bytes, thread::priority(1), core3);

      kernel::start();

      EXPECT_EQ(s.consumed.load(), tokens_total) << "consumed token count diverged from minted";
      EXPECT_EQ(s.final_peek.load(), 0u)         << "count nonzero after exact consumption";
      EXPECT_FALSE(s.post_drain_take.load())     << "try_acquire granted an unminted token";

      kernel::finalise();
   }
}

/* ============================================================================
 * release(n) frees n parked waiters
 *
 * Three waiters park on a zero semaphore, one per core, each attested by a
 * same-core lower-priority witness. A single release(3) must let all three
 * complete. Completing at all is the wake-sufficiency proof (an under-waking
 * release hangs the stragglers and the framework timeout catches it), and
 * the count arithmetic proves no waiter consumed more than one token.
 * ========================================================================= */
TEST_F(SyncSemaphore_Test, GivenThreeParkedWaiters_WhenReleaseThree_ThenAllProceedOnExactlyOneTokenEach)
{
   for (int rep = 0; rep < contract_reps; ++rep) {
      SCOPED_TRACE("multi-wake rep " + std::to_string(rep));

      kernel::initialise();

      static std::array<aligned_stack, 7> stacks{};

      struct state
      {
         semaphore sem{0};
         std::atomic<int> parked{0};
         std::atomic<int> proceeded{0};
         std::atomic<std::size_t> final_peek{~std::size_t{0}};
      } s;

      constexpr core_affinity waiter_core[3] = { core0, core1, core2 };

      std::array<thread, 3> waiters{};
      std::array<thread, 3> witnesses{};
      for (std::size_t i = 0; i < 3; ++i) {
         waiters[i] = thread(
            [&s]{
               s.sem.acquire();
               s.proceeded.fetch_add(1, std::memory_order_acq_rel);
            },
            stacks[i].bytes, thread::priority(1), waiter_core[i]);

         // Lower priority on the waiter's own core: proof of park.
         witnesses[i] = thread(
            [&s]{ s.parked.fetch_add(1, std::memory_order_acq_rel); },
            stacks[3 + i].bytes, thread::priority(2), waiter_core[i]);
      }

      thread releaser(
         [&s]{
            while (s.parked.load(std::memory_order_acquire) < 3) {
               cyros_port_cpu_relax();
            }
            s.sem.release(3);
            while (s.proceeded.load(std::memory_order_acquire) < 3) {
               cyros_port_cpu_relax();
            }
            s.final_peek.store(s.sem.peek(), std::memory_order_release);
         },
         stacks[6].bytes, thread::priority(0), core3);

      kernel::start();

      EXPECT_EQ(s.proceeded.load(), 3)   << "release(3) did not free all three parked waiters";
      EXPECT_EQ(s.final_peek.load(), 0u) << "waiters consumed a token count other than three";

      kernel::finalise();
   }
}