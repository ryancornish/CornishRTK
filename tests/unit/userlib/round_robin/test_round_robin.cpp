#include <cyros/rr/round_robin.hpp>
#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>
#include <cyros/port/port_traits.h>
#include <cyros/config/config.hpp>

#include "gtest/gtest.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

using namespace cyros;

static_assert(config::cores == 1, "Test suite is designed for single core configuration only");

static constexpr auto STACK_SIZE = thread::min_stack_size + (32 * 1024);

namespace
{

alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> bootstrap_stack{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> a_stack{};
alignas(CYROS_PORT_STACK_ALIGN) std::array<std::byte, STACK_SIZE> b_stack{};

constexpr std::size_t num_workers = 5;
struct alignas(CYROS_PORT_STACK_ALIGN) worker_stack { std::array<std::byte, STACK_SIZE> bytes; };
std::array<worker_stack, num_workers> worker_stacks{};

}  // namespace

/*
 * Round-robin proof-of-life on the real preempt timer.
 *
 * Two equal-priority threads each run a tight, non-yielding spin. On a single
 * core only one runs at a time, so the ONLY thing that can hand the core to the
 * other is an asynchronous timer interrupt rotating them. That is why this test
 * is backed by a real time driver (tickless) on linux_preempt, not simulation:
 * a spinner never pumps virtual time, so under simulation the slice timer would
 * never fire and the threads would run to completion one after the other.
 *
 * The witness is a_saw_b: thread A records whether it ever observed B making
 * progress while A itself was still running. That can only be true if the slice
 * timer rotated the core from A to B and back while A was mid-run, which is
 * exactly round-robin. The target is chosen well above one slice worth of
 * iterations so at least one rotation is forced before either thread finishes.
 */
TEST(RoundRobin_Test,
     GivenTwoEqualPriorityThreads_WhenRoundRobinTimerIsActivated_ThenBothThreadsRotate)
{
   kernel::initialise();

   // GIVEN: a spin long enough to span many 1 ms slices.
   constexpr std::uint64_t target = 10'000'000;

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};
   std::atomic<bool>          a_saw_b{false};

   thread bootstrap(
      []()
      {
         rr::enable_round_robin(time::from_milliseconds(1));
         time::start();
      },
      bootstrap_stack,
      thread::priority(0),
      core0
   );

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

   thread a(
      [&]{ bounded_spin(a_count, b_count, &a_saw_b); },
      a_stack,
      thread::priority(1),
      core0
   );

   thread b(
      [&]{ bounded_spin(b_count, a_count, nullptr); },
      b_stack,
      thread::priority(1),
      core0
   );

   // WHEN: tickless time, a 1 ms slice enabled before release (it pends and is
   // anchored at time::start()), then time and the kernel go live.
   time::initialise(1'000'000);          // 1 MHz, matches the port clock

   kernel::start();

   // THEN: both ran to completion, and A saw B progress mid-run, which is only
   // possible if the slice timer rotated them.
   EXPECT_EQ(a_count.load(), target);
   EXPECT_EQ(b_count.load(), target);
   EXPECT_TRUE(a_saw_b.load());

   time::finalise();
   kernel::finalise();
}

class RoundRobinCpuMeasure : public ::testing::TestWithParam<uint32_t>
{
private:
   static constexpr std::uint32_t total_work_base = 10'000;

protected:
   static constexpr std::uint32_t frequency = 1'000'000; // 1 MHz, matches the port clock

   uint32_t quantum_us{0};
   uint32_t total_work{0};

   void SetUp() override
   {
      quantum_us = GetParam();
      // Scale work done with infrequency of rotates to witness similar amounts of rotates
      total_work = total_work_base * quantum_us;
      kernel::initialise();
      time::initialise(frequency);
   }

   void TearDown() override
   {
      time::finalise();
      kernel::finalise();
   }
};

TEST_P(RoundRobinCpuMeasure,
       GivenTwoEqualPriorityThreads_WhenRoundRobinEnabled_ThenCpuShareIsFair)
{
   std::atomic<std::uint64_t> work{0};

   std::atomic<std::uint64_t> a_count{0};
   std::atomic<std::uint64_t> b_count{0};

   // -1 = nobody yet
   //  0 = A last executed
   //  1 = B last executed
   std::atomic<int> last_thread{-1};

   std::atomic<std::uint64_t> switches_to_a{0};
   std::atomic<std::uint64_t> switches_to_b{0};

   thread bootstrap(
      [&]()
      {
         rr::enable_round_robin(time::from_microseconds(quantum_us));
         time::start();
      },
      bootstrap_stack,
      thread::priority(0),
      core0
   );

   auto worker =
      [&](int id,
          std::atomic<std::uint64_t>& my_count,
          std::atomic<std::uint64_t>& my_switches)
   {
      while (true)
      {
         // Detect that the other thread ran since we last executed.
         if (last_thread.exchange(id, std::memory_order_relaxed) != id)
         {
            ++my_switches;
         }

         auto index = work.fetch_add(1, std::memory_order_relaxed);

         if (index >= total_work)
         {
            break;
         }

         ++my_count;
      }
   };

   thread a(
      [&]
      {
         worker(0, a_count, switches_to_a);
      },
      a_stack,
      thread::priority(1),
      core0
   );

   thread b(
      [&]
      {
         worker(1, b_count, switches_to_b);
      },
      b_stack,
      thread::priority(1),
      core0
   );

   auto const start = time::now();
   kernel::start();
   auto const elapsed = duration_between(time::now(), start);

   auto const a_res = a_count.load();
   auto const b_res = b_count.load();

   double const a_share = 100.0 * double(a_res) / double(total_work);
   double const b_share = 100.0 * double(b_res) / double(total_work);

   auto const total_switches = switches_to_a.load() + switches_to_b.load();
   auto const elapsed_ms = time::to_milliseconds(elapsed);

   auto const expected_switches = elapsed.value / time::from_microseconds(quantum_us).value;

   std::cout << "\n";
   std::cout << "Test parameters:\n";
   std::cout << "- Frequency      : " << frequency << " hz\n";
   std::cout << "- Quantum        : " << quantum_us << " us\n";
   std::cout << "- Work           : " << total_work << " counts\n";
   std::cout << "Test results   :\n";
   std::cout << "- Elapsed        : " << elapsed_ms << " ms\n";
   std::cout << "- A count        : " << a_res << "\n";
   std::cout << "- B count        : " << b_res << "\n";
   std::cout << "- A share        : " << a_share << "%\n";
   std::cout << "- B share        : " << b_share << "%\n";
   std::cout << "- Switches -> A  : " << switches_to_a.load() << "\n";
   std::cout << "- Switches -> B  : " << switches_to_b.load() << "\n";
   std::cout << "- Total switches : " << total_switches << "\n";
   std::cout << "- Expected ~     : " << expected_switches << "\n";

   EXPECT_EQ(a_res + b_res, total_work);

   // Round-robin's actual contract is that two equal-priority threads on one
   // core alternate, so the transition counts into each can differ by at most one
   
   auto const switches_a = switches_to_a.load();
   auto const switches_b = switches_to_b.load();
   auto const switch_imbalance =
      switches_a > switches_b ? switches_a - switches_b : switches_b - switches_a;
   EXPECT_LE(switch_imbalance, 2u);

   // CPU share is a far weaker signal than the switch balance above, because it
   // measures work completed per slice rather than scheduling decisions
   EXPECT_NEAR(a_share, 50.0, 10.0);
   EXPECT_NEAR(b_share, 50.0, 10.0);

   // Sanity check that RR actually occurred.
   EXPECT_GT(total_switches, 10u);
}

std::string QuantumName(testing::TestParamInfo<uint32_t> const& info)
{
   return std::to_string(info.param) + "us";
}

INSTANTIATE_TEST_SUITE_P(
   QuantumTests,
   RoundRobinCpuMeasure,
   ::testing::Values(100, 500, 1000, 5000, 10'000),
   QuantumName
);



/*
 * Round-robin fair-share across FIVE equal-priority threads.
 *
 * The two-thread sweep proves rotation happens and is even in aggregate, but with
 * two threads "rotate behind peers" and "alternate A/B" are the same motion, so a
 * FIFO bug and a correct rotation look identical. With five, they diverge: correct
 * round-robin must cycle through all of them (A->B->C->D->E->A), and a bug that
 * ping-pongs a subset while starving the rest shows up as skew in BOTH per-thread
 * CPU share and per-thread schedule count.
 *
 * This test is a WITNESS, not a strict gate. It prints the per-thread breakdown
 * and hard-asserts only the robust invariants: all work is accounted for,
 * rotation actually occurred, and no thread is grossly starved. Read the report
 * to see how even the split really is; the tight fairness numbers are for the eye,
 * not the CI threshold, because five-way splitting plus the finishing tail makes
 * strict bounds inappropriate.
 */
TEST(RoundRobin_Test,
     GivenFiveEqualPriorityThreads_WhenRoundRobinEnabled_ThenCpuShareIsFair)
{
   constexpr std::uint32_t frequency  = 1'000'000; // 1 MHz, matches the port clock
   constexpr std::uint32_t quantum_us = 1'000;     // 1 ms slice
   constexpr std::uint64_t total_work = 10'000'000;

   kernel::initialise();
   time::initialise(frequency);

   std::atomic<std::uint64_t> work{0};

   std::array<std::atomic<std::uint64_t>, num_workers> counts{};
   std::array<std::atomic<std::uint64_t>, num_workers> schedules{};

   // Which worker ran most recently. A worker seeing a value other than its own
   // id means it was just rotated in, i.e. a schedule-in for that worker.
   std::atomic<int> last_thread{-1};

   thread bootstrap(
      [&]()
      {
         rr::enable_round_robin(time::from_microseconds(quantum_us));
         time::start();
      },
      bootstrap_stack,
      thread::priority(0),
      core0
   );

   auto worker = [&](int id)
   {
      auto const me = static_cast<std::size_t>(id);
      while (true)
      {
         // A transition into us (someone else ran since our last turn) counts as
         // a schedule-in. Summed over workers this is total switches; per worker
         // it is how often each was rotated in, which is the rotation-evenness
         // witness the two-thread test could not provide.
         if (last_thread.exchange(id, std::memory_order_relaxed) != id)
         {
            schedules[me].fetch_add(1, std::memory_order_relaxed);
         }

         auto const index = work.fetch_add(1, std::memory_order_relaxed);
         if (index >= total_work)
         {
            break;
         }

         counts[me].fetch_add(1, std::memory_order_relaxed);
      }
   };

   std::vector<thread> workers;
   workers.reserve(num_workers);
   for (std::size_t i = 0; i < num_workers; ++i)
   {
      workers.emplace_back(
         [&worker, i]{ worker(static_cast<int>(i)); },
         worker_stacks[i].bytes,
         thread::priority(1),
         core0
      );
   }

   auto const start = time::now();
   kernel::start();
   auto const elapsed = duration_between(time::now(), start);

   // ---- gather ----
   std::uint64_t total_counted  = 0;
   std::uint64_t total_switches = 0;
   for (std::size_t i = 0; i < num_workers; ++i)
   {
      total_counted  += counts[i].load();
      total_switches += schedules[i].load();
   }

   auto const elapsed_ms         = time::to_milliseconds(elapsed);
   auto const expected_switches  = elapsed.value / time::from_microseconds(quantum_us).value;
   double const fair_share       = 100.0 / double(num_workers);

   // ---- report ----
   std::cout << "\n";
   std::cout << "Test parameters:\n";
   std::cout << "- Frequency      : " << frequency << " hz\n";
   std::cout << "- Quantum        : " << quantum_us << " us\n";
   std::cout << "- Workers        : " << num_workers << "\n";
   std::cout << "- Work           : " << total_work << " counts\n";
   std::cout << "Test results   :\n";
   std::cout << "- Elapsed        : " << elapsed_ms << " ms\n";
   for (std::size_t i = 0; i < num_workers; ++i)
   {
      double const share = 100.0 * double(counts[i].load()) / double(total_work);
      std::cout << "- Thread " << i << "        : "
                << counts[i].load() << " counts, "
                << share << "% share, "
                << schedules[i].load() << " schedules\n";
   }
   std::cout << "- Total switches : " << total_switches << "\n";
   std::cout << "- Expected ~     : " << expected_switches << "\n";
   std::cout << "- Fair share     : " << fair_share << "% each\n";

   // ---- hard invariants ----
   EXPECT_EQ(total_counted, total_work);   // all work accounted for
   EXPECT_GT(total_switches, 20u);         // rotation actually happened

   // ---- soft fairness witness: catch only gross starvation ----
   // Generous band on purpose. This guards against one thread being starved or
   // hogging the core, not against normal five-way variance.
   for (std::size_t i = 0; i < num_workers; ++i)
   {
      double const share = 100.0 * double(counts[i].load()) / double(total_work);
      EXPECT_NEAR(share, fair_share, 10.0)
         << "thread " << i << " share is far from fair - possible starvation";
   }

   time::finalise();
   kernel::finalise();
}
