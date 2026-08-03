/**
 * @file test_linux_coop_port.cpp
 * @brief Unit tests for port layer (boost.context backend)
 */

#include <cyros/port/port.h>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>


static_assert(CYROS_PORT_CORE_COUNT > 0);

// ----------------------------------------------------------------------------
// Test harness: per-core scheduler loop driven by port->kernel hook.
// ----------------------------------------------------------------------------


// Global pointer used by the C reschedule hook installed via cyros_port_init().
static constinit struct PortHarness* g_harness = nullptr;

struct PortHarness
{
   static constexpr std::size_t MAX_READY = 32;

   // Simple single-core ready queue (FIFO). Enough for port-level tests.
   std::array<cyros_port_context_t*, MAX_READY> ready{};
   std::size_t r_head{0};
   std::size_t r_tail{0};

   // Track what the harness believes is currently running (optional / informational).
   cyros_port_context_t* current{nullptr};

   void reset() noexcept
   {
      r_head = r_tail = 0;
      current = nullptr;
      ready.fill(nullptr);
   }

   bool empty() const noexcept { return r_head == r_tail; }

   std::size_t size() const noexcept { return r_tail - r_head; }

   void enqueue(cyros_port_context_t* ctx)
   {
      ASSERT_NE(ctx, nullptr);
      ASSERT_LT(size(), MAX_READY) << "Ready queue overflow in test harness";
      ready[r_tail % MAX_READY] = ctx;
      ++r_tail;
   }

   cyros_port_context_t* dequeue() noexcept
   {
      if (empty()) return nullptr;
      auto* ctx = ready[r_head % MAX_READY];
      ready[r_head % MAX_READY] = nullptr;
      ++r_head;
      return ctx;
   }

   // Called by the port via cyros_port_pend_reschedule_handler.
   void reschedule_once()
   {
      auto* next = dequeue();
      if (!next) return;

      // In these unit tests we always resume threads from the test's OS fiber.
      // That means the "caller" context captured for each thread is the test thread,
      // and cyros_port_pend_reschedule() returns control back here.
      if (!current) {
         current = next;
         cyros_port_start_first(next);
         // When the thread yields/exits, we return here.
         // `current` is only "best effort" bookkeeping; the thread may have exited.
         current = nullptr;
         return;
      }

      auto* prev = current;
      current = next;
      cyros_port_switch(prev, next);
      current = nullptr;
   }

   // Drive until no ready threads remain (or step limit hit).
   void run_until_quiescent(std::size_t step_limit = 10'000)
   {
      for (std::size_t i = 0; i < step_limit; ++i) {
         if (empty()) return;
         reschedule_once();
      }
      FAIL() << "Harness did not quiesce within step limit (possible missing yield/enqueue)";
   }

   // Hook trampoline.
   static void reschedule_hook()
   {
      ASSERT_NE(g_harness, nullptr);
      g_harness->reschedule_once();
   }
};

struct ThreadArg
{
   PortHarness* harness{};
   cyros_port_context_t* ctx{};
   std::atomic<int>* stage_counter{};
   int stages{};
   int thread_id{};
   std::array<int, 64>* trace{};
   std::atomic<int>* trace_len{};
};

// A thread that records its "thread_id" into trace, yields `stages` times, then returns.
static void thread_yield_n(void* varg)
{
   auto* a = static_cast<ThreadArg*>(varg);
   ASSERT_NE(a, nullptr);
   ASSERT_NE(a->harness, nullptr);
   ASSERT_NE(a->ctx, nullptr);

   for (int i = 0; i < a->stages; ++i) {
      // Record progress
      int idx = a->trace_len->fetch_add(1, std::memory_order_acq_rel);
      ASSERT_LT(idx, static_cast<int>(a->trace->size()));
      (*a->trace)[static_cast<std::size_t>(idx)] = a->thread_id;

      a->stage_counter->fetch_add(1, std::memory_order_acq_rel);

      // Cooperative yield: put ourselves back on the ready queue, then yield to caller.
      a->harness->enqueue(a->ctx);
      cyros_port_pend_reschedule();
   }

   // Return = thread exits; port test harness regains control.
}

// A thread that runs once and returns (no yield).
static void thread_run_once(void* varg)
{
   auto* ran = static_cast<std::atomic<bool>*>(varg);
   ASSERT_NE(ran, nullptr);
   ran->store(true, std::memory_order_release);
}

// Thread for the deferred-reschedule tests. Records a trace of "events":
// it pends a reschedule while preemption is disabled, records that it is
// STILL running (proving the switch was deferred), releases preemption, then
// records once more. A peer thread, if scheduled, stamps the trace between.
struct DeferArg
{
   PortHarness*       harness{};
   cyros_port_context_t* self_ctx{};
   cyros_port_context_t* peer_ctx{};   // may be nullptr
   std::array<int, 16>* trace{};
   std::atomic<int>*    trace_len{};
   int                  id{};
   bool                 peer_already_enqueued{false};
};

static void defer_record(DeferArg* a, int marker)
{
   int idx = a->trace_len->fetch_add(1, std::memory_order_acq_rel);
   ASSERT_LT(idx, static_cast<int>(a->trace->size()));
   (*a->trace)[static_cast<std::size_t>(idx)] = marker;
}

// Entry: pends a reschedule while preemption is disabled, and proves the
// switch does not happen until cyros_port_preempt_enable() is called.
static void thread_defer_probe(void* varg)
{
   auto* a = static_cast<DeferArg*>(varg);
   ASSERT_NE(a, nullptr);

   if (a->peer_ctx && !a->peer_already_enqueued) {
      a->harness->enqueue(a->peer_ctx);
   }

   cyros_mask_token_t t = cyros_port_preempt_disable();
   defer_record(a, 100 + a->id);

   cyros_port_pend_reschedule();          // deferred - no switch yet
   defer_record(a, 200 + a->id);           // still running - deferral proven

   // Re-enqueue self BEFORE the safe point, so the deferred switch has a
   // path back to us. (Cooperative-harness discipline: a thread that wants
   // to run again must be on the ready queue before it yields control.)
   a->harness->enqueue(a->self_ctx);

   cyros_port_preempt_enable(t);          // deferred reschedule resolves here
   defer_record(a, 300 + a->id);
}

// A peer that simply stamps the trace and exits.
static void thread_defer_peer(void* varg)
{
   auto* a = static_cast<DeferArg*>(varg);
   ASSERT_NE(a, nullptr);
   defer_record(a, 900 + a->id);          // marker: peer ran
}

class PortTest : public ::testing::Test
{
protected:
   PortHarness port_harness{};

   void SetUp() override
   {
      port_harness.reset();
      g_harness = &port_harness;
      cyros_port_init(&PortHarness::reschedule_hook);
   }

   void TearDown() override
   {
      g_harness = nullptr;
   }
};



TEST_F(PortTest, GivenIrqSaveRestore_WhenNested_ThenRestoreReturnsToPriorState)
{
   EXPECT_TRUE(cyros_port_interrupts_enabled());

   // The token is opaque: obtained from save, handed back to the matching
   // restore, never inspected. Correctness is observed via interrupts_enabled(),
   // not the token's value.
   cyros_mask_token_t s0 = cyros_port_irq_save();
   EXPECT_FALSE(cyros_port_interrupts_enabled());

   cyros_mask_token_t s1 = cyros_port_irq_save();
   EXPECT_FALSE(cyros_port_interrupts_enabled());

   // Restore the inner level: still disabled, because the outer save holds it.
   cyros_port_irq_restore(s1);
   EXPECT_FALSE(cyros_port_interrupts_enabled());

   // Restore the outer level: now back to the original enabled state.
   cyros_port_irq_restore(s0);
   EXPECT_TRUE(cyros_port_interrupts_enabled());
}

TEST_F(PortTest, GivenTlsPointer_WhenSetThenGet_ThenValueRoundTrips)
{
   int x = 123;
   cyros_port_set_tls_pointer(&x);
   EXPECT_EQ(cyros_port_get_tls_pointer(), &x);

   cyros_port_set_tls_pointer(nullptr);
   EXPECT_EQ(cyros_port_get_tls_pointer(), nullptr);
}

TEST_F(PortTest, GivenGetCoreId_WhenNotStartedCores_ThenIsZero)
{
   // In the Linux coop port, core_id defaults to 0 on the calling thread unless start_cores sets it.
   EXPECT_EQ(cyros_port_get_core_id(), 0u);
}

TEST_F(PortTest, GivenSingleThread_WhenStartFirst_ThenThreadRunsAndReturnsToCaller)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> stack{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> ctx_storage{};

   auto* ctx = reinterpret_cast<cyros_port_context_t*>(ctx_storage.data());

   std::atomic<bool> ran{false};

   cyros_port_context_init(
      ctx,
      stack.data(),
      stack.size(),
      &thread_run_once,
      &ran
   );

   // Schedule it manually via the harness
   port_harness.enqueue(ctx);
   port_harness.run_until_quiescent();

   EXPECT_TRUE(ran.load(std::memory_order_acquire));

   // Thread should have completed and returned control to us.
   // (The port's context_destroy is owned by the kernel normally, but port-level test can still destroy.)
   cyros_port_context_destroy(ctx);
}

TEST_F(PortTest, GivenTwoThreads_WhenTheyYieldAndReenqueue_ThenTheyInterleaveInFIFOOrder)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s0{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s1{};

   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c0_storage{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c1_storage{};

   auto* c0 = reinterpret_cast<cyros_port_context_t*>(c0_storage.data());
   auto* c1 = reinterpret_cast<cyros_port_context_t*>(c1_storage.data());

   std::atomic<int> stages_done{0};
   std::atomic<int> trace_len{0};
   std::array<int, 64> trace{};
   trace.fill(-1);

   ThreadArg a0{
      .harness = &port_harness,
      .ctx = c0,
      .stage_counter = &stages_done,
      .stages = 3,
      .thread_id = 0,
      .trace = &trace,
      .trace_len = &trace_len,
   };

   ThreadArg a1{
      .harness = &port_harness,
      .ctx = c1,
      .stage_counter = &stages_done,
      .stages = 3,
      .thread_id = 1,
      .trace = &trace,
      .trace_len = &trace_len,
   };

   cyros_port_context_init(c0, s0.data(), s0.size(), &thread_yield_n, &a0);
   cyros_port_context_init(c1, s1.data(), s1.size(), &thread_yield_n, &a1);

   // Enqueue in a known order
   port_harness.enqueue(c0);
   port_harness.enqueue(c1);

   // Drive until both threads finish all stages.
   // Each stage yields and reenqueues itself, so the harness should alternate FIFO.
   port_harness.run_until_quiescent();

   EXPECT_EQ(stages_done.load(std::memory_order_acquire), 6);
   EXPECT_EQ(trace_len.load(std::memory_order_acquire), 6);

   // Expected: 0,1,0,1,0,1
   EXPECT_EQ(trace[0], 0);
   EXPECT_EQ(trace[1], 1);
   EXPECT_EQ(trace[2], 0);
   EXPECT_EQ(trace[3], 1);
   EXPECT_EQ(trace[4], 0);
   EXPECT_EQ(trace[5], 1);

   cyros_port_context_destroy(c0);
   cyros_port_context_destroy(c1);
}

/* ============================================================================
 * Deferred reschedule contract (Preemption Control)
 *
 * These exercise the mechanism the kernel-side preemption refactor introduced:
 * cyros_port_pend_reschedule() must DEFER when preemption is disabled, and the
 * deferred reschedule must RESOLVE at cyros_port_preempt_enable() depth 0.
 * ========================================================================= */

// pend_reschedule() with preemption disabled must NOT switch immediately;
// the switch is observed only after preempt_enable().
TEST_F(PortTest, GivenPreemptionDisabled_WhenPendReschedule_ThenSwitchDeferredUntilEnable)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_probe{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_peer{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_probe{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_peer{};

   auto* probe = reinterpret_cast<cyros_port_context_t*>(c_probe.data());
   auto* peer  = reinterpret_cast<cyros_port_context_t*>(c_peer.data());

   std::array<int, 16> trace{};
   trace.fill(-1);
   std::atomic<int> trace_len{0};

   DeferArg probe_arg{
      .harness = &port_harness, .self_ctx = probe, .peer_ctx = peer,
      .trace = &trace, .trace_len = &trace_len, .id = 0,
   };
   DeferArg peer_arg{
      .harness = &port_harness, .self_ctx = peer, .peer_ctx = nullptr,
      .trace = &trace, .trace_len = &trace_len, .id = 0,
   };

   cyros_port_context_init(probe, s_probe.data(), s_probe.size(), &thread_defer_probe, &probe_arg);
   cyros_port_context_init(peer,  s_peer.data(),  s_peer.size(),  &thread_defer_peer,  &peer_arg);

   port_harness.enqueue(probe);
   port_harness.run_until_quiescent();

   // Expected order:
   //   100  probe entered critical section
   //   200  probe STILL running after pend_reschedule (deferral proven)
   //   900  peer ran - only after preempt_enable released the deferral
   //   300  probe resumed after peer finished
   ASSERT_EQ(trace_len.load(), 4);
   EXPECT_EQ(trace[0], 100);
   EXPECT_EQ(trace[1], 200) << "peer ran before preempt_enable - deferral failed";
   EXPECT_EQ(trace[2], 900) << "peer did not run at preempt_enable - resolution failed";
   EXPECT_EQ(trace[3], 300);

   cyros_port_context_destroy(probe);
   cyros_port_context_destroy(peer);
}

// Nested preemption disable: the deferred reschedule must wait for the
// OUTERMOST preempt_enable (depth returning to 0), not an inner one.
TEST_F(PortTest, GivenNestedPreemptDisable_WhenPendReschedule_ThenResolvesOnlyAtOutermostEnable)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_probe{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_peer{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_probe{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_peer{};

   auto* probe = reinterpret_cast<cyros_port_context_t*>(c_probe.data());
   auto* peer  = reinterpret_cast<cyros_port_context_t*>(c_peer.data());

   std::array<int, 16> trace{};
   trace.fill(-1);
   std::atomic<int> trace_len{0};

   // The probe entry below is inlined as a lambda-free static; reuse DeferArg.
   DeferArg probe_arg{
      .harness = &port_harness, .self_ctx = probe, .peer_ctx = peer,
      .trace = &trace, .trace_len = &trace_len, .id = 1,
   };
   DeferArg peer_arg{
      .harness = &port_harness, .self_ctx = peer, .peer_ctx = nullptr,
      .trace = &trace, .trace_len = &trace_len, .id = 1,
   };

   // Custom entry for the nested case.
   auto nested_entry = +[](void* varg)
   {
      auto* a = static_cast<DeferArg*>(varg);
      a->harness->enqueue(a->peer_ctx);

      cyros_mask_token_t t1 = cyros_port_preempt_disable();  // depth 1
      cyros_mask_token_t t2 = cyros_port_preempt_disable();  // depth 2
      defer_record(a, 100 + a->id);

      cyros_port_pend_reschedule();          // deferred

      cyros_port_preempt_enable(t2);         // depth 1 - still masked, no switch
      defer_record(a, 200 + a->id);

      a->harness->enqueue(a->self_ctx);

      cyros_port_preempt_enable(t1);         // depth 0 - switch resolves here
      defer_record(a, 300 + a->id);
   };

   cyros_port_context_init(probe, s_probe.data(), s_probe.size(), nested_entry, &probe_arg);
   cyros_port_context_init(peer,  s_peer.data(),  s_peer.size(),  &thread_defer_peer, &peer_arg);

   port_harness.enqueue(probe);
   port_harness.run_until_quiescent();

   // 101 entered (depth 2); 201 still running after inner enable (depth 1);
   // 901 peer ran at outermost enable; 301 probe resumed.
   ASSERT_EQ(trace_len.load(), 4);
   EXPECT_EQ(trace[0], 101);
   EXPECT_EQ(trace[1], 201) << "switch happened at inner preempt_enable - depth tracking broken";
   EXPECT_EQ(trace[2], 901);
   EXPECT_EQ(trace[3], 301);

   cyros_port_context_destroy(probe);
   cyros_port_context_destroy(peer);
}

// Baseline sanity: with preemption ENABLED, pend_reschedule resolves
// immediately - the peer runs before the line after pend_reschedule.
TEST_F(PortTest, GivenBaselinePriority_WhenPendReschedule_ThenSwitchIsImmediate)
{
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_probe{};
   alignas(CYROS_PORT_STACK_ALIGN) static std::array<std::byte, 16 * 1024> s_peer{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_probe{};
   alignas(CYROS_PORT_CONTEXT_ALIGN) static std::array<std::byte, CYROS_PORT_CONTEXT_SIZE> c_peer{};

   auto* probe = reinterpret_cast<cyros_port_context_t*>(c_probe.data());
   auto* peer  = reinterpret_cast<cyros_port_context_t*>(c_peer.data());

   std::array<int, 16> trace{};
   trace.fill(-1);
   std::atomic<int> trace_len{0};

   DeferArg probe_arg{
      .harness = &port_harness, .self_ctx = probe, .peer_ctx = peer,
      .trace = &trace, .trace_len = &trace_len, .id = 2,
   };
   DeferArg peer_arg{
      .harness = &port_harness, .self_ctx = peer, .peer_ctx = nullptr,
      .trace = &trace, .trace_len = &trace_len, .id = 2,
   };

   auto baseline_entry = +[](void* varg)
   {
      auto* a = static_cast<DeferArg*>(varg);
      a->harness->enqueue(a->peer_ctx);

      defer_record(a, 100 + a->id);           // running, no critical section
      a->harness->enqueue(a->self_ctx);
      cyros_port_pend_reschedule();          // baseline -> resolves NOW
      defer_record(a, 300 + a->id);           // resumed after peer
   };

   cyros_port_context_init(probe, s_probe.data(), s_probe.size(), baseline_entry, &probe_arg);
   cyros_port_context_init(peer,  s_peer.data(),  s_peer.size(),  &thread_defer_peer, &peer_arg);

   port_harness.enqueue(probe);
   port_harness.run_until_quiescent();

   // 102 entered; 902 peer ran immediately (before 302); 302 probe resumed.
   ASSERT_EQ(trace_len.load(), 3);
   EXPECT_EQ(trace[0], 102);
   EXPECT_EQ(trace[1], 902) << "peer did not run - baseline pend_reschedule was not immediate";
   EXPECT_EQ(trace[2], 302);

   cyros_port_context_destroy(probe);
   cyros_port_context_destroy(peer);
}

TEST_F(PortTest, GivenCpuRelax_WhenCalled_ThenDoesNotCrash)
{
   // Not asserting timing/behaviour; just that it's callable.
   cyros_port_cpu_relax();
   cyros_port_cpu_relax();
}

TEST_F(PortTest, GivenGetStackPointer_WhenCalled_ThenReturnsNonNull)
{
   void* sp = cyros_port_get_stack_pointer();
   EXPECT_NE(sp, nullptr);
}

TEST_F(PortTest, Asserts)
{
   CYROS_ASSERT(true);
   CYROS_ASSERT1(true == true, 30);
   CYROS_ASSERT2(!false, 30, 20);
   CYROS_ASSERT_OP(1, <, 2);
   int* i_am_null = nullptr;
   CYROS_ASSERT_NULL(i_am_null);
}

// Uncomment each line to test kernel panics
TEST_F(PortTest, MakesError)
{
   // CYROS_ASSERT(false); // ...Some handy error diagnosis comment...
   // CYROS_ASSERT1(true == false, 30);
   // CYROS_ASSERT2(!true, 30, 20); // ...Some handy error diagnosis comment...
   // CYROS_ASSERT_OP(1, >, 2);  // ...Some handy error diagnosis comment...
   // int x; CYROS_ASSERT_NULL(&x);  // ...Some handy error diagnosis comment...
}

