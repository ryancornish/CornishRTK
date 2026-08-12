/**
 * @file test_sync_mutex_pi_chain.cpp
 * @brief Deep transitive priority-inheritance chains, parameterised on depth.
 *
 * WHY THIS EXISTS
 * ---------------
 * Every other PI test tops out at a chain of depth 2, which is the shallowest
 * shape that is transitive at all. Two consequences that this file addresses:
 *
 * 1. The transitive chase has been broken for as long as it has existed, and it
 *    survived precisely because almost nothing exercised it. Depth 2 once per
 *    test run is not pressure.
 * 2. The cost of evaluating a thread's urgency is bounded by chain DEPTH, and
 *    depth is the one quantity the substrate stage 2 probe could not measure,
 *    because it only ever saw flat folds. See pi-derived-urgency-proposal.md
 *    section 11b.
 *
 * MEASURED 2026-08-06, AGAINST THE OLD PUSH DESIGN (HISTORICAL)
 * -------------------------------------------------------------
 * These numbers predate derived urgency and describe a mechanism that has since
 * been deleted, including the transitive bug they were taken to characterise.
 * Kept because the LATENCY shape is the interesting part and nobody has
 * re-measured it against the fold: the cost model changed from one cross-core
 * doorbell round trip per link to one recursive fold per pick, which should
 * change this curve substantially. Re-running the cost curve is the obvious
 * follow-up.
 *
 *   depth curve, 200 rounds per depth, 1000 rounds total in 3.5 seconds:
 *
 *       depth | fail% | min spins | mean spins
 *           2 |  0.5% |         1 |        913
 *           3 |  0.0% |         1 |       1145
 *           4 |  0.5% |        12 |       1652
 *           5 |  0.0% |       208 |       2121
 *           6 |  0.5% |       605 |       3260
 *
 *       Two things fall out. Propagation LATENCY grows with depth, and the min
 *       column is the clean signal because it is the floor with scheduling
 *       variance removed: 1, 1, 12, 208, 605. That is what a serialised chain of
 *       cross-core doorbells predicts, one reschedule round trip per link.
 *
 *       Failure RATE does not grow with depth: flat at about 0.3 percent overall
 *       and unrelated to chain length. So the transitive bug is a timing window
 *       at a single link, not risk accumulating over links.
 *   repeated depth-3 (50 rounds per run): fails about 1 round in 250, showing
 *       up in roughly 8 runs out of 40.
 *
 * So the known transitive bug IS reproduced here, but at ~0.4 percent per round
 * against ~14 percent per run for the hand-written depth-2 test in
 * test_sync_mutex_pi.cpp. That test remains the better trigger.
 *
 * The likely reason, and it is worth knowing before extending this file: every
 * gate below is a semaphore, so the chain is strictly ordered and fully blocked
 * before the donor parks. The failing window in the real bug is a thread that is
 * ARMED but NOT YET BLOCKED, which clean sequencing largely avoids. A future
 * variant that has the donor park CONCURRENTLY with a mid-chain thread blocking
 * would attack that window directly, and is the obvious next iteration.
 *
 * ENABLED 2026-08-09
 * -------------------
 * The two assertion tests ran DISABLED for as long as the transitive chase was
 * broken, because a permanently red suite destroys the thing every conclusion in
 * that investigation depended on: being able to run a few hundred iterations and
 * read the result. That bug is closed, and not by fixing the chase but by
 * deleting it, see sync-mutex-pi-plan.md section 0. Re-enabling them was the
 * documented completion bar.
 *
 * Evidence for enabling, laptop, at the commit that closed it:
 *   filtered to these tests   0 failures / 40 runs
 *   full binary               0 chain failures / ~150 runs
 * against a recorded baseline of roughly 8 runs in 40 for the repeated depth-3
 * test alone.
 *
 * The COST CURVE test stays disabled, and not because it is flaky. It reports
 * rather than asserts, the number being the deliverable, and 200 rounds across
 * five depths does not belong in a suite people run in a loop. Run it
 * deliberately:
 *   ./test_sync_mutex --gtest_also_run_disabled_tests --gtest_filter='*CostCurve*'
 *
 * Be aware it has produced a genuine KERNEL PANIC once in 40 runs, a signal-mask
 * mismatch at port_linux_preempt.cpp:430, which is a PORT defect this file
 * merely stresses rather than anything about chaining. Worth knowing before you
 * blame the chain code for it.
 *
 * THE SHAPE
 * ---------
 * For depth D, holder[i] owns mutex[i], and holder[i] for i >= 1 then blocks on
 * mutex[i-1]. An urgent thread finally blocks on mutex[D-1]. So one donation at
 * the top must chase D links to reach holder[0]:
 *
 *   urgent -> holder[D-1] -> holder[D-2] -> ... -> holder[0]
 *
 * Holders are spread round-robin across cores so most links are cross-core,
 * which is the case the propagation actually has to survive.
 */

#include <cyros/sync/mutex.hpp>
#include <cyros/sync/semaphore.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/config/config.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/port/port.h>

#include "gtest/gtest.h"

#include <chrono>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

using namespace cyros;
static_assert(config::cores >= 4, "Chain tests assume at least a quad-core configuration");

namespace
{

constexpr std::size_t max_depth = 6;   // holders; +1 urgent, +1 conductor

/* A chain of this many holders consults a queue that still has a bridge on it
 * at recursion depth 0, so the fold gives up and over-boosts. Worked from
 * max_inheritance_depth = 8: queue m[k] is reached at depth 8-k, so depth 0
 * lands on m[8], and m[8] only has a bridge when there are 10 or more holders.
 * At 9 the chain still resolves exactly. Capacity, not part of the sweep. */
constexpr std::size_t over_budget_depth = 10;
constexpr std::size_t max_threads = over_budget_depth + 2;

constexpr auto CHAIN_STACK_SIZE = thread::min_stack_size + (16 * 1024);

// Deliberately larger than the flat tests use. A deep chain needs D cross-core
// doorbell round trips to settle, serialised, so the budget has to scale with
// depth or a slow-but-correct chain reads as a failure.
constexpr std::uint64_t chain_poll_budget = 60'000'000;

struct alignas(CYROS_PORT_STACK_ALIGN) chain_stack
{
   std::array<std::byte, CHAIN_STACK_SIZE> bytes;
};

/// @brief Spin until predicate or budget. Returns iterations used, or 0 on failure.
template <typename Predicate>
[[nodiscard]] std::uint64_t poll_counting(std::uint64_t const budget, Predicate&& done) noexcept
{
   for (std::uint64_t i = 1; i <= budget; ++i) {
      if (done()) return i;
      cyros_port_cpu_relax();
   }
   return 0;
}

/// @brief Outcome of one chain run, so the caller can assert or report cost.
struct chain_result
{
   bool          formed{false};        // every holder acquired and the chain built
   bool          propagated{false};    // every holder reached the urgent priority
   bool          restored{false};      // every holder returned to its base
   std::uint64_t propagate_spins{0};   // proxy for end-to-end propagation latency
   std::uint8_t  worst_observed{0};    // least urgent priority seen at the check

   /* Cost of ONE urgency fold, nanoseconds, measured while the chain is fully
    * formed and every participant is parked, so the queue locks are uncontended.
    * deep_ns folds the whole chain (the deepest holder, one queue lock and one
    * recursion per link). flat_ns is the same call on a thread holding nothing,
    * which is the overwhelmingly common case and the pick's old cost. */
   double        deep_ns{0.0};
   double        flat_ns{0.0};
};

/* Kept out of the optimiser's reach. get_priority is [[nodiscard]] and lives
 * out of line, but the loop below is otherwise dead code. */
std::atomic<unsigned> fold_sink{0};

/**
 * @brief Build a depth-D chain, donate once at the top, observe the far end.
 *
 * Base priorities descend with index so holder[0] is least urgent and every
 * intermediate donation is a real change rather than a no-op. The urgent thread
 * sits at 1, so a fully propagated chain leaves EVERY holder at 1.
 */
/* The budget is a parameter because the cost of ONE check scales with depth:
 * each get_priority() is a recursive fold that walks the chain taking a queue
 * lock per link. A budget tuned for a depth-3 chain, spent on a depth-10 one,
 * turns a failing assertion into a 60-second timeout, which is the failure mode
 * this suite least wants. */
chain_result run_chain(std::size_t depth, std::uint64_t poll_budget = chain_poll_budget,
                       bool measure_fold = false) noexcept
{
   chain_result result{};

   static std::array<chain_stack, max_threads> stacks{};

   // Rebuilt per run. A failing chain does not necessarily leave its mutexes
   // free, and a failing chain is exactly what this file expects.
   struct fixtures
   {
      std::array<mutex, over_budget_depth>            m{};
      std::array<sync::semaphore, over_budget_depth>  link; // link[i]: holder[i] owns m[i]
      sync::semaphore                         formed{0}; // chain is fully armed
      sync::semaphore                         release{0};// conductor says unwind
      sync::semaphore                         done{0};   // one post per holder exit

      /* sync::semaphore has no default constructor, so every element has to be
       * named. The static_assert is the guard: raise over_budget_depth without
       * adding initialisers here and this fails to compile rather than silently
       * leaving the tail of the array unbuilt. */
      static_assert(over_budget_depth == 10, "add or remove link initialisers below to match");
      fixtures() : link{sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
                        sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
                        sync::semaphore{0}, sync::semaphore{0}, sync::semaphore{0},
                        sync::semaphore{0}} {}
   };
   static std::optional<fixtures> fx;
   fx.emplace();

   struct state
   {
      fixtures*                      f{nullptr};
      std::array<thread*, over_budget_depth> holder{};
      std::size_t                    depth{0};
      std::uint64_t                  budget{chain_poll_budget};
      bool                           measure{false};
      chain_result*                  out{nullptr};
   };
   static state s;

   for (auto& h : s.holder) h = nullptr;
   s.f      = &*fx;
   s.depth  = depth;
   s.budget = poll_budget;
   s.measure = measure_fold;
   s.out    = &result;

   // holder[0] is the deepest and least urgent. Urgent is 1, conductor 0.
   auto const base_of = [depth](std::size_t i) -> std::uint8_t {
      return static_cast<std::uint8_t>(depth + 2 - i);
   };
   auto const core_of = [](std::size_t i) {
      switch (i % 4) {
         case 0:  return core0;
         case 1:  return core1;
         case 2:  return core2;
         default: return core3;
      }
   };

   std::array<std::optional<thread>, max_threads> threads{};

   /* EVERY gate below is a semaphore, never a spin flag. With more chain links
    * than cores some threads necessarily share a core, and a spinning waiter
    * starves the very thread it is waiting for. That deadlocks the harness and
    * reads as a kernel hang, which is exactly the confusion this file exists to
    * avoid. A blocked waiter yields its core, so sharing is harmless. */

   for (std::size_t i = 0; i < depth; ++i) {
      threads[i].emplace(
         [i]{
            auto& f = *s.f;
            if (i > 0) f.link[i - 1].acquire();   // the link below now exists

            f.m[i].lock();
            f.link[i].release();                  // arm the link above us

            if (i > 0) {
               f.m[i - 1].lock();   // blocks: this is our link of the chain
               f.m[i - 1].unlock(); // reached only as the chain unwinds
            } else {
               f.release.acquire(); // deepest holder waits for the conductor
            }

            f.m[i].unlock();
            f.done.release();
         },
         stacks[i].bytes,
         thread::priority(base_of(i)),
         core_of(i));
      s.holder[i] = &threads[i].value();
   }

   // The donor: parks on the TOP of the chain, so its urgency must travel every
   // link to reach holder[0].
   threads[depth].emplace(
      [depth]{
         auto& f = *s.f;
         f.link[depth - 1].acquire();
         f.formed.release();      // conductor may start observing
         f.m[depth - 1].lock();
         f.m[depth - 1].unlock();
      },
      stacks[depth].bytes,
      thread::priority(1),
      core_of(depth));

   // Conductor. Blocked until the chain is armed, so it never starves a builder.
   threads[depth + 1].emplace(
      [depth]{
         auto& f = *s.f;
         auto& r = *s.out;

         f.formed.acquire();
         r.formed = true;

         // Safe to spin here and only here: every holder is blocked on a mutex
         // and the donor is blocked on the top of the chain, so this thread is
         // the only runnable one on whatever core it shares.
         // <= rather than ==. Over-boosting is legal: past the fold's depth
         // budget a link answers 0, which is MORE urgent than the donor and is
         // the deliberate safe direction. An exact-match poll would spin its
         // whole budget on a chain that is behaving correctly.
         auto const all_urgent = [depth]{
            for (std::size_t i = 0; i < depth; ++i) {
               if (s.holder[i]->get_priority() > thread::priority(1)) return false;
            }
            return true;
         };
         r.propagate_spins = poll_counting(s.budget, all_urgent);
         r.propagated      = (r.propagate_spins != 0);

         /* Cost of the fold, measured here because the chain is formed, fully
          * propagated and completely parked, so nothing contends the queue
          * locks. This is the pick's new term: pick_next folds urgency for
          * every holder on its core, and holder[0]'s fold is the worst case
          * because it walks every link. */
         if (r.propagated && s.measure) {
            constexpr int fold_samples = 2000;
            unsigned sink = 0;

            auto const t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < fold_samples; ++i) {
               sink += static_cast<unsigned>(s.holder[0]->get_priority());
            }
            auto const t1 = std::chrono::steady_clock::now();

            // Same call on a thread holding nothing: the fast path, one load
            // and a compare, and what the pick cost before any of this.
            for (int i = 0; i < fold_samples; ++i) {
               sink += static_cast<unsigned>(this_thread::priority());
            }
            auto const t2 = std::chrono::steady_clock::now();

            fold_sink.fetch_add(sink, std::memory_order_relaxed);
            r.deep_ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / fold_samples;
            r.flat_ns = std::chrono::duration<double, std::nano>(t2 - t1).count() / fold_samples;
         }

         // How far the chase actually got. On failure this says which link it
         // died at, which is the useful number.
         std::uint8_t worst = 0;
         for (std::size_t i = 0; i < depth; ++i) {
            auto const p = static_cast<std::uint8_t>(s.holder[i]->get_priority());
            if (p > worst) worst = p;
         }
         r.worst_observed = worst;

         f.release.release();
         for (std::size_t i = 0; i < depth; ++i) f.done.acquire();
         r.restored = true;
      },
      stacks[depth + 1].bytes,
      thread::priority(0),
      core_of(depth + 1));

   kernel::start();
   kernel::finalise();
   return result;
}

class SyncMutexPiChain_Test : public ::testing::Test {};

}  // namespace


/* ============================================================================
 * Correctness: one donation must reach the far end of a D-deep chain
 * ========================================================================= */

TEST_F(SyncMutexPiChain_Test, GivenChainOfIncreasingDepth_WhenUrgentWaiterParksAtTop_ThenEveryLinkInherits)
{
   for (std::size_t depth = 2; depth <= max_depth; ++depth) {
      SCOPED_TRACE("chain depth " + std::to_string(depth));

      kernel::initialise();
      auto const r = run_chain(depth);

      EXPECT_TRUE(r.formed) << "chain never formed, so the rest proves nothing";
      EXPECT_TRUE(r.propagated)
         << "donation did not reach every link. Least urgent priority still seen: "
         << int(r.worst_observed) << " (expected 1). A value between 1 and the "
            "base priorities says which link the chase died at.";
      EXPECT_TRUE(r.restored) << "chain did not unwind";
   }
}

/* ============================================================================
 * Past the fold's depth budget, a chain must OVER-boost and never under-boost
 *
 * The urgency fold recurses through bridges with a budget of
 * max_inheritance_depth (8). The budget exists for wait-for CYCLES, i.e. a
 * deadlocked application, which must not be followed forever. A long ACYCLIC
 * chain hits the same limit, and this test uses that because a cycle cannot be
 * torn down: deadlocked threads never terminate, so the kernel never quiesces
 * and the test could not finish. A 10-deep chain reaches the same code path and
 * still unwinds cleanly.
 *
 * The assertion is one-directional, exactly as the bridge-snapshot overflow test
 * in test_sync_mutex_pi_derived.cpp is. Over-boosting past the budget is the
 * designed behaviour and costs bounded fairness. Under-boosting is unbounded
 * priority inversion, which is what the whole subsystem exists to prevent. So
 * this asserts only that no link is LESS urgent than the donor, and never that
 * some link reads exactly 0.
 *
 * That also keeps it correct if max_inheritance_depth is later raised: the chain
 * would then resolve exactly instead of over-boosting, and every holder would
 * read 1, which still satisfies the assertion for the right reason.
 * ========================================================================= */

TEST_F(SyncMutexPiChain_Test, GivenChainDeeperThanTheFoldsBudget_WhenDonating_ThenNoLinkUnderBoosts)
{
   kernel::initialise();
   auto const r = run_chain(over_budget_depth, 200'000);

   EXPECT_TRUE(r.formed) << "chain never formed, so the rest proves nothing";
   EXPECT_TRUE(r.propagated)
      << "a link past the depth budget is LESS urgent than the donor. Least "
         "urgent priority seen: " << int(r.worst_observed)
      << ", donor is at 1. Exhausting the budget must answer 0 and over-boost, "
         "because under-reporting here is unbounded inversion.";
   EXPECT_TRUE(r.restored) << "chain did not unwind";
}


/* ============================================================================
 * Pressure: hammer the shallow-but-transitive case repeatedly
 *
 * Depth 3 is the cheapest shape that needs a genuine second hop. Repeating it
 * many times inside one binary is the closest thing to a stress test for the
 * chase, and it is what a rarely-exercised path needs.
 * ========================================================================= */

TEST_F(SyncMutexPiChain_Test, GivenRepeatedShallowChains_WhenDonatingManyTimes_ThenEveryRoundPropagates)
{
   constexpr int rounds = 50;
   int failures = 0;

   for (int round = 0; round < rounds; ++round) {
      kernel::initialise();
      auto const r = run_chain(3);
      if (!r.propagated) ++failures;
   }

   EXPECT_EQ(failures, 0) << failures << " of " << rounds
                          << " rounds failed to propagate a depth-3 chain";
}

/* ============================================================================
 * Measurement: what does depth actually cost
 *
 * Reports rather than asserts, because the number is the deliverable. Answers
 * the question substrate stage 2 could not: how propagation latency scales with
 * chain depth. Under the current push design each link is a cross-core doorbell
 * plus a reschedule, so expect roughly linear growth. Under derived urgency the
 * same measurement prices the recursive fold instead.
 * ========================================================================= */

/* ============================================================================
 * What one urgency fold costs, as a function of chain depth
 *
 * The pick used to be a bitmap scan. It is now a bitmap scan plus, for every
 * holder ready on that core, a fold that can take a queue spinlock and recurse
 * once per link, all inside the retire-and-pick critical section with local
 * interrupts disabled. cross-core-defects.md 8.2 states that bound. This
 * measures it, because principle 5 asks for a number and a stated bound is not
 * a measured one.
 *
 * Reports two columns. FLAT is a thread holding nothing, which is the fast path
 * and 98.9 percent of picks. DEEP is the chain's deepest holder, whose fold
 * walks every link and is the worst case the depth budget permits.
 *
 * Measured with the whole chain parked, so the locks are UNCONTENDED. That is
 * the floor, not the ceiling: a fold racing other cores on the same queues pays
 * more. Report it as such.
 *
 * Measurement only, no assertion, so it stays disabled and is run deliberately:
 *   ./test_sync_mutex --gtest_also_run_disabled_tests --gtest_filter='*FoldCost*'
 * ========================================================================= */
TEST_F(SyncMutexPiChain_Test, DISABLED_GivenIncreasingDepth_WhenFoldingUrgency_ThenReportFoldCost)
{
   constexpr int rounds_per_depth = 25;

   std::fprintf(stderr,
      "\ndepth | samples | flat ns | deep ns | per-link ns\n");
   std::fprintf(stderr,
      "------|---------|---------|---------|------------\n");

   double flat_at_two = 0.0;
   double deep_at_two = 0.0;

   for (std::size_t depth = 2; depth <= max_depth; ++depth) {
      double flat_sum = 0.0;
      double deep_sum = 0.0;
      int    ok       = 0;

      for (int round = 0; round < rounds_per_depth; ++round) {
         kernel::initialise();
         auto const r = run_chain(depth, chain_poll_budget, /*measure_fold=*/true);
         if (r.propagated && r.deep_ns > 0.0) {
            flat_sum += r.flat_ns;
            deep_sum += r.deep_ns;
            ++ok;
         }
      }

      if (ok == 0) {
         std::fprintf(stderr, "%5zu | %7d |       - |       - |           -\n", depth, ok);
         continue;
      }

      double const flat = flat_sum / ok;
      double const deep = deep_sum / ok;
      if (depth == 2) {
         flat_at_two = flat;
         deep_at_two = deep;
      }
      // Slope from depth 2, which is the marginal cost of one more link.
      double const per_link = (depth > 2)
         ? (deep - deep_at_two) / static_cast<double>(depth - 2)
         : 0.0;

      std::fprintf(stderr, "%5zu | %7d | %7.1f | %7.1f | %11.1f\n",
                   depth, ok, flat, deep, per_link);
   }

   std::fprintf(stderr,
      "\nflat is a thread holding nothing (the common pick). deep folds the whole\n"
      "chain. Uncontended: every participant is parked while this is measured.\n");
   if (flat_at_two > 0.0) {
      std::fprintf(stderr, "ratio deep(2)/flat = %.1fx\n", deep_at_two / flat_at_two);
   }
   std::fflush(stderr);

   SUCCEED() << "measurement only, see the table on stderr";
}

TEST_F(SyncMutexPiChain_Test, DISABLED_GivenIncreasingDepth_WhenMeasuringPropagation_ThenReportCostCurve)
{
   // Each depth is sampled many times rather than once. A single sample per
   // depth is too low-powered to say anything about scaling, and it also cannot
   // see whether the known transitive flake gets worse as the chain lengthens,
   // which is the more interesting question of the two.
   constexpr int rounds_per_depth = 200;

   std::fprintf(stderr,
      "\ndepth | rounds | failed | fail%% | min spins | mean spins | max spins\n");
   std::fprintf(stderr,
      "------|--------|--------|-------|-----------|------------|----------\n");

   for (std::size_t depth = 2; depth <= max_depth; ++depth) {
      int           failed = 0;
      std::uint64_t lo     = ~std::uint64_t{0};
      std::uint64_t hi     = 0;
      std::uint64_t sum    = 0;
      int           ok     = 0;

      for (int round = 0; round < rounds_per_depth; ++round) {
         kernel::initialise();
         auto const r = run_chain(depth);
         if (!r.propagated) {
            ++failed;
            continue;
         }
         ++ok;
         sum += r.propagate_spins;
         if (r.propagate_spins < lo) lo = r.propagate_spins;
         if (r.propagate_spins > hi) hi = r.propagate_spins;
      }

      std::fprintf(stderr, "%5zu | %6d | %6d | %4.1f%% | %9llu | %10.1f | %8llu\n",
                   depth, rounds_per_depth, failed,
                   100.0 * static_cast<double>(failed) / rounds_per_depth,
                   static_cast<unsigned long long>(ok ? lo : 0),
                   ok ? static_cast<double>(sum) / static_cast<double>(ok) : 0.0,
                   static_cast<unsigned long long>(hi));
   }
   std::fflush(stderr);

   SUCCEED() << "measurement only, see the table on stderr";
}
