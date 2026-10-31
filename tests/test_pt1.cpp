#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <fc/math/constants.h>
#include <fc/pt1.h>

namespace {

constexpr float kSampleRateHz = 4000.0f;
constexpr float kDt = 1.0f / kSampleRateHz;  // period in seconds
constexpr float kCutoffHz = 100.0f;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Expected alpha, computed from the formula independently of the
// implementation, so a bug in setCutoff() can't cancel out in the test.
float expectedAlpha(float cutoffHz, float dt) {
    const float tau = 1.0f / (2.0f * fc::kPi * cutoffHz);
    return dt / (tau + dt);
}

// Feed a constant input n times and return the last output.
float runConstant(fc::Pt1& f, float input, int n) {
    float y = f.value();
    for (int i = 0; i < n; ++i) {
        y = f.apply(input);
    }
    return y;
}

}  // namespace

TEST(Pt1, StepResponseMatchesClosedForm) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 0.0f));
    const float alpha = expectedAlpha(kCutoffHz, kDt);

    // Step 0 -> 1: after n samples, y[n] = 1 - (1 - alpha)^n.
    for (int n = 1; n <= 200; ++n) {
        const float y = f.apply(1.0f);
        const float expected = 1.0f - std::pow(1.0f - alpha, static_cast<float>(n));
        EXPECT_NEAR(static_cast<double>(y), static_cast<double>(expected), 1e-5)
            << "at sample n=" << n;
    }
}

TEST(Pt1, ConvergesToConstantInput) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 0.0f));
    // tau = 1/(2*pi*100) = ~1.6 ms = ~6.4 samples, so 1000 samples is ~150 tau.
    const float y = runConstant(f, 1.0f, 1000);
    EXPECT_NEAR(static_cast<double>(y), 1.0, 1e-4);
}

TEST(Pt1, InitSetsInitialOutput) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 3.0f));
    EXPECT_FLOAT_EQ(f.value(), 3.0f);
}

TEST(Pt1, ResetRemovesStartupTransient) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 0.0f));
    f.reset(5.0f);
    // Input equal to the reset value: output must not move.
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(f.apply(5.0f), 5.0f);
    }
}

TEST(Pt1, StepNeverOvershootsAndIsMonotonic) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 0.0f));
    float previous = 0.0f;
    for (int i = 0; i < 500; ++i) {
        const float y = f.apply(1.0f);
        EXPECT_GE(y, previous) << "output went backwards at i=" << i;
        EXPECT_LE(y, 1.0f) << "overshoot at i=" << i;
        previous = y;
    }
}

TEST(Pt1, InitRejectsInvalidCutoff) {
    fc::Pt1 f;
    EXPECT_FALSE(f.init(0.0f, kDt, 0.0f));
    EXPECT_FALSE(f.init(-10.0f, kDt, 0.0f));
    EXPECT_FALSE(f.init(kNaN, kDt, 0.0f));
    EXPECT_FALSE(f.init(2500.0f, kDt, 0.0f));  // above Nyquist (2000 Hz)
}

TEST(Pt1, InitRejectsInvalidDt) {
    fc::Pt1 f;
    EXPECT_FALSE(f.init(kCutoffHz, 0.0f, 0.0f));
    EXPECT_FALSE(f.init(kCutoffHz, -kDt, 0.0f));
    EXPECT_FALSE(f.init(kCutoffHz, kNaN, 0.0f));
}

TEST(Pt1, InitAcceptsCutoffJustBelowNyquist) {
    fc::Pt1 f;
    EXPECT_TRUE(f.init(1900.0f, kDt, 0.0f));
}

TEST(Pt1, FailedInitDoesNotChangeOutput) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 2.0f));
    ASSERT_FALSE(f.init(-1.0f, kDt, 99.0f));
    EXPECT_FLOAT_EQ(f.value(), 2.0f);
}

TEST(Pt1, FailedSetCutoffLeavesCoefficientUnchanged) {
    fc::Pt1 a;
    fc::Pt1 b;
    ASSERT_TRUE(a.init(kCutoffHz, kDt, 0.0f));
    ASSERT_TRUE(b.init(kCutoffHz, kDt, 0.0f));

    // Several invalid calls on 'a' only.
    ASSERT_FALSE(a.setCutoff(0.0f, kDt));
    ASSERT_FALSE(a.setCutoff(kNaN, kDt));
    ASSERT_FALSE(a.setCutoff(5000.0f, kDt));

    // If k had changed, the two filters would diverge on the same input.
    for (int i = 0; i < 50; ++i) {
        EXPECT_FLOAT_EQ(a.apply(1.0f), b.apply(1.0f)) << "diverged at i=" << i;
    }
}

TEST(Pt1, DefaultConstructedFilterIsInert) {
    // Documents current behavior: before init(), k == 0, so output never moves.
    fc::Pt1 f;
    EXPECT_FLOAT_EQ(f.value(), 0.0f);
    EXPECT_FLOAT_EQ(f.apply(1.0f), 0.0f);
}

TEST(Pt1, StepAfterOneTimeConstantIsRoughlyTextbook63Percent) {
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kCutoffHz, kDt, 0.0f));

    // tau/dt = 1/w = ~6.37 samples, so the nearest whole sample is n = 6.
    const float tau = 1.0f / (2.0f * fc::kPi * kCutoffHz);
    const int n = static_cast<int>(std::lround(tau / kDt));
    ASSERT_EQ(n, 6);

    const float y = runConstant(f, 1.0f, n);

    // Textbook: 1 - 1/e = 0.632 at t = tau. We get ~0.583 because of two effects:
    //  1. n = 6 is slightly before t = tau (6.37 samples): continuous value ~0.610.
    //  2. Backward Euler's pole is 1/(1+w) = 0.864, slower than the exact
    //     e^-w = 0.855, which pulls 0.610 down to ~0.583.
    // Total gap ~0.049, so a 0.07 tolerance is a justified "behaves like the
    // textbook filter" check. Exactness is covered by the closed-form test.
    EXPECT_NEAR(static_cast<double>(y), 0.632, 0.07);
}

TEST(Pt1, OneStepNearNyquistUsesLargeAlpha) {
    constexpr float kHighCutoffHz = 1900.0f;  // just below Nyquist (2000 Hz)
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kHighCutoffHz, kDt, 0.0f));

    // From 0 with a unit step, one update returns exactly alpha (~0.749).
    // Note it is nowhere near 1: backward Euler caps alpha at ~0.76.
    const float expected = expectedAlpha(kHighCutoffHz, kDt);
    EXPECT_NEAR(static_cast<double>(f.apply(1.0f)),
                static_cast<double>(expected), 1e-5);
    EXPECT_LT(expected, 0.76f);
}

TEST(Pt1, OneStepAtVeryLowCutoffBarelyMoves) {
    constexpr float kLowCutoffHz = 0.1f;
    fc::Pt1 f;
    ASSERT_TRUE(f.init(kLowCutoffHz, kDt, 0.0f));

    // alpha ~1.57e-4. A relative tolerance is needed, since an absolute 1e-5
    // would be ~6% of the value.
    const double expected = static_cast<double>(expectedAlpha(kLowCutoffHz, kDt));
    EXPECT_NEAR(static_cast<double>(f.apply(1.0f)), expected, expected * 1e-4);
}