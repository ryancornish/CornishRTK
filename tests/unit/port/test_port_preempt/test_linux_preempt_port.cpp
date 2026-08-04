/**
 * @file test_linux_preempt_port.cpp
 * @brief Port-contract tests for the linux_preempt port.
 *
 * Sibling of test_linux_coop_port.cpp, pinned to port = "linux_preempt" so it
 * runs under the preempt profile and skips elsewhere.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The coop port test covered interrupt- and preempt-disable regions only when
 * STRICTLY NESTED. Those are exactly the two orderings that always worked. An
 * ownership-token bug in the linux_preempt port lived for weeks in the two
 * OVERLAPPING orderings, which no test exercised: both regions would reach depth
 * zero with the reschedule signal still masked, leaving the core permanently
 * unpreemptible, and thread-exit's final reschedule never fired.
 *
 * WHAT IS OBSERVED, AND WHY IT IS THE RIGHT THING
 * -----------------------------------------------
 * cyros_port_interrupts_enabled() cannot see this class of bug: it reports a
 * depth counter, and in the leaked state both depths were legitimately zero. It
 * returned true while the signal was still masked.
 *
 * So these tests assert the property that actually matters and that a depth
 * counter cannot fake: A BALANCED SEQUENCE OF DISABLE/ENABLE PAIRS MUST LEAVE THE
 * THREAD'S SIGNAL MASK EXACTLY AS IT FOUND IT. That is signal-agnostic, so the
 * test never needs to know which signals the port owns, and it catches both
 * failure directions: a signal left masked (the leak), and a signal unmasked too
 * early and then re-masked (which is what a naive absolute SIG_SETMASK of a saved
 * token does, and is worse, since a reschedule then fires inside a critical
 * section).
 *
 * Tokens are treated as opaque throughout, matching the port contract: obtain one
 * from a disable, hand it to the matching enable, never inspect it.
 */

#include <cyros/port/port.h>

#include <gtest/gtest.h>

#include <csignal>
#include <string>
#include <vector>

namespace
{

/// @brief Every signal number this platform can mask, for whole-mask comparison.
std::vector<int> maskable_signals()
{
   std::vector<int> signals;
   for (int sig = 1; sig < NSIG; ++sig) {
      // SIGKILL and SIGSTOP cannot be blocked, so they never appear in a mask.
      if (sig == SIGKILL || sig == SIGSTOP) continue;
      signals.push_back(sig);
   }
   return signals;
}

sigset_t current_mask()
{
   sigset_t live;
   sigemptyset(&live);
   pthread_sigmask(SIG_BLOCK, nullptr, &live);
   return live;
}

/// @brief Names the signals that differ, so a failure says what leaked.
std::string mask_difference(sigset_t const& before, sigset_t const& after)
{
   std::string report;
   for (int const sig : maskable_signals()) {
      bool const was = sigismember(&before, sig) != 0;
      bool const now = sigismember(&after, sig) != 0;
      if (was == now) continue;
      report += " signal " + std::to_string(sig)
              + (now ? " left BLOCKED but was deliverable" : " left DELIVERABLE but was blocked")
              + ";";
   }
   return report;
}

void reschedule_hook() {}

class PreemptPortTest : public ::testing::Test
{
protected:
   void SetUp() override { cyros_port_init(&reschedule_hook); }
};

} // namespace


/* ============================================================================
 * Region bookkeeping: the mask must survive any balanced sequence
 *
 * Four orderings of one interrupt region (I) and one preempt region (P). The two
 * nested cases are the ones the coop test already covered on its own port. The
 * two overlapping cases are the regression tests for the token-ownership bug.
 * ========================================================================= */

TEST_F(PreemptPortTest, GivenNestedInterruptOuter_WhenRegionsClose_ThenMaskIsRestored)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   cyros_mask_token_t const i = cyros_port_irq_save();
   EXPECT_FALSE(cyros_port_interrupts_enabled());
   cyros_mask_token_t const p = cyros_port_preempt_disable();

   cyros_port_preempt_enable(p);
   EXPECT_FALSE(cyros_port_interrupts_enabled()) << "interrupt region still open";
   cyros_port_irq_restore(i);

   EXPECT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const after = current_mask();
   EXPECT_EQ(mask_difference(before, after), "") << "I(P) nesting did not restore the mask";
}

TEST_F(PreemptPortTest, GivenNestedPreemptOuter_WhenRegionsClose_ThenMaskIsRestored)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   cyros_mask_token_t const p = cyros_port_preempt_disable();
   cyros_mask_token_t const i = cyros_port_irq_save();
   EXPECT_FALSE(cyros_port_interrupts_enabled());

   cyros_port_irq_restore(i);
   cyros_port_preempt_enable(p);

   EXPECT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const after = current_mask();
   EXPECT_EQ(mask_difference(before, after), "") << "P(I) nesting did not restore the mask";
}

// REGRESSION: interrupt region opened first and closed first, while a preempt
// region opened inside it is still live. The old scheme left the reschedule
// signal masked here, because irq_restore correctly declined to reopen it and
// preempt_enable then believed the reopen was never its business.
TEST_F(PreemptPortTest, GivenOverlappingInterruptFirst_WhenBothClose_ThenMaskIsRestored)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   cyros_mask_token_t const i = cyros_port_irq_save();
   cyros_mask_token_t const p = cyros_port_preempt_disable();

   cyros_port_irq_restore(i);   // closes first, but P is still open
   EXPECT_TRUE(cyros_port_interrupts_enabled()) << "interrupt depth did not clear";
   cyros_port_preempt_enable(p);

   sigset_t const after = current_mask();
   EXPECT_EQ(mask_difference(before, after), "")
      << "overlapping I-then-P left the mask changed. Neither region reopened what"
         " the other declined to, which is the ownership bug this test exists for.";
}

// REGRESSION: the mirror image. Preempt region opened first and closed first
// while an interrupt region opened inside it is still live.
TEST_F(PreemptPortTest, GivenOverlappingPreemptFirst_WhenBothClose_ThenMaskIsRestored)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   cyros_mask_token_t const p = cyros_port_preempt_disable();
   cyros_mask_token_t const i = cyros_port_irq_save();

   cyros_port_preempt_enable(p); // closes first, but I is still open
   EXPECT_FALSE(cyros_port_interrupts_enabled()) << "interrupt region still open";
   cyros_port_irq_restore(i);

   EXPECT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const after = current_mask();
   EXPECT_EQ(mask_difference(before, after), "")
      << "overlapping P-then-I left the mask changed";
}

// The masking must be idempotent and order-independent, so repeating the
// overlapping sequences must not accumulate drift. A scheme that leaks one signal
// per pass shows up immediately here even if a single pass looked clean.
TEST_F(PreemptPortTest, GivenOverlappingRegionsRepeated_WhenManyPasses_ThenNoMaskDrift)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   for (int pass = 0; pass < 64; ++pass) {
      cyros_mask_token_t const i = cyros_port_irq_save();
      cyros_mask_token_t const p = cyros_port_preempt_disable();
      cyros_port_irq_restore(i);
      cyros_port_preempt_enable(p);

      cyros_mask_token_t const p2 = cyros_port_preempt_disable();
      cyros_mask_token_t const i2 = cyros_port_irq_save();
      cyros_port_preempt_enable(p2);
      cyros_port_irq_restore(i2);

      ASSERT_EQ(mask_difference(before, current_mask()), "")
         << "mask drifted on pass " << pass;
   }

   EXPECT_TRUE(cyros_port_interrupts_enabled());
}

// Deep nesting of the same region type must only reopen at the outermost close.
TEST_F(PreemptPortTest, GivenDeeplyNestedInterruptRegions_WhenUnwound_ThenOnlyOutermostReopens)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());
   sigset_t const before = current_mask();

   std::vector<cyros_mask_token_t> tokens;
   for (int depth = 0; depth < 8; ++depth) {
      tokens.push_back(cyros_port_irq_save());
      EXPECT_FALSE(cyros_port_interrupts_enabled()) << "at depth " << depth;
   }

   while (tokens.size() > 1) {
      cyros_port_irq_restore(tokens.back());
      tokens.pop_back();
      EXPECT_FALSE(cyros_port_interrupts_enabled())
         << "reopened at depth " << tokens.size() << " instead of the outermost";
   }

   cyros_port_irq_restore(tokens.back());
   EXPECT_TRUE(cyros_port_interrupts_enabled());
   EXPECT_EQ(mask_difference(before, current_mask()), "");
}

// interrupts_enabled() reports the interrupt depth only. Preempt-disable must not
// move it, because an ISR still runs under preempt-disable and only the switch it
// requests is deferred. This pins the interrupt-versus-preempt split.
TEST_F(PreemptPortTest, GivenPreemptDisabled_WhenQueried_ThenInterruptsStillReportEnabled)
{
   ASSERT_TRUE(cyros_port_interrupts_enabled());

   cyros_mask_token_t const p = cyros_port_preempt_disable();
   EXPECT_TRUE(cyros_port_interrupts_enabled())
      << "preempt-disable must not raise the interrupt depth";

   cyros_port_preempt_enable(p);
   EXPECT_TRUE(cyros_port_interrupts_enabled());
}
