// Kept rate within +-5% of the configured rate under input jitter.
#include <gtest/gtest.h>

#include <random>

#include "nav_data_recorder/throttle.hpp"

using nav_data_recorder::Throttle;

static double keptRate(double in_hz, double out_hz, double jitter_frac, double seconds)
{
  Throttle t(out_hz);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> j(-jitter_frac, jitter_frac);
  const double period = 1.0 / in_hz;
  int kept = 0;
  const int n = static_cast<int>(seconds * in_hz);
  for (int i = 0; i < n; ++i) {
    const double ts = i * period + j(rng) * period;
    if (t.accept(static_cast<int64_t>(ts * 1e9))) {
      ++kept;
    }
  }
  return kept / seconds;
}

TEST(Throttle, TfLike) {EXPECT_NEAR(keptRate(500, 50, 0.3, 60), 50, 2.5);}
TEST(Throttle, OdomLike) {EXPECT_NEAR(keptRate(100, 20, 0.3, 60), 20, 1.0);}
TEST(Throttle, ScanLike) {EXPECT_NEAR(keptRate(30, 10, 0.2, 60), 10, 0.5);}
TEST(Throttle, PathLike) {EXPECT_NEAR(keptRate(20, 5, 0.2, 60), 5, 0.25);}
TEST(Throttle, SlowerInputKeepsAll) {EXPECT_NEAR(keptRate(2, 5, 0.1, 60), 2, 0.1);}
// A 2 Hz costmap gated at 2 Hz must not lose messages to receive jitter
TEST(Throttle, SameRateKeepsAll) {EXPECT_NEAR(keptRate(2, 2, 0.1, 120), 2, 0.05);}
TEST(Throttle, SameRateHighJitter) {EXPECT_NEAR(keptRate(10, 10, 0.15, 60), 10, 0.5);}
TEST(Throttle, NeverAboveRate)
{
  for (double in : {2.2, 2.5, 3.0, 4.0, 5.0, 7.0, 11.0}) {
    EXPECT_LE(keptRate(in, 2, 0.15, 120), 2.0 * 1.01) << in;
    EXPECT_GE(keptRate(in, 2, 0.15, 120), 2.0 * 0.95) << in;
  }
}

TEST(Throttle, Disabled)
{
  Throttle t(0.0);
  EXPECT_FALSE(t.enabled());
  EXPECT_TRUE(t.accept(1));
  EXPECT_TRUE(t.accept(1));
}

TEST(Throttle, ClockBackResets)
{
  Throttle t(10.0);
  EXPECT_TRUE(t.accept(10'000'000'000));
  EXPECT_FALSE(t.accept(10'050'000'000));
  EXPECT_TRUE(t.accept(1'000'000'000));  // sim restart
}
