#include <gtest/gtest.h>

#include <limits>

#include <fc/math/constants.h>
#include <fc/pid.h>

namespace {

constexpr float kDt = 1.0f / 4000.0f;  // 4 kHz loop, period in seconds
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

double toD(float x) { return static_cast<double>(x); }

// Same alpha formula as the Pt1 tests, computed independently of the code.
float expectedAlpha(float cutoffHz, float dt) {
    const float tau = 1.0f / (2.0f * fc::kPi * cutoffHz);
    return dt / (tau + dt);
}

// Configure everything and reset. Returns false if any setter rejected.
// (Not void, so callers wrap it in ASSERT_TRUE.)
bool configure(fc::Pid& p, float kp, float ki, float kd,
               float iLimit = 100.0f, float dCutoffHz = 100.0f,
               float kff = 0.0f, float ffCutoffHz = 100.0f) {
    const bool ok = p.setGains(kp, ki, kd, kDt, kff) && p.setILimit(iLimit) &&
                    p.setFilteredD(dCutoffHz, kDt) &&
                    p.setFilteredFF(ffCutoffHz, kDt) &&
                    p.setOutLimit(1000.0f);
    p.reset(0.0f, 0.0f);
    return ok;
}

// Drive two pids with the same input and expect identical outputs.
void expectSameBehavior(fc::Pid& a, fc::Pid& b) {
    for (int i = 0; i < 100; ++i) {
        const float m = 0.01f * static_cast<float>(i);
        EXPECT_FLOAT_EQ(a.update(1.0f, m), b.update(1.0f, m))
            << "diverged at i=" << i;
    }
}

}  // namespace

// ---------- P ----------

TEST(Pid, ProportionalOnly) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 2.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 2.0f);
    EXPECT_FLOAT_EQ(p.update(0.5f, 1.0f), -1.0f);  // negative error
}

TEST(Pid, ZeroErrorGivesZeroOutput) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 2.0f, 10.0f, 0.01f));
    p.reset(3.0f, 3.0f);
    EXPECT_FLOAT_EQ(p.update(3.0f, 3.0f), 0.0f);
}

// ---------- I ----------

TEST(Pid, IntegratorAccumulatesAtExpectedRate) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 10.0f, 0.0f, 100.0f));
    // Constant error 1: after n steps output = ki * dt * n = 0.0025 * n.
    for (int n = 1; n <= 200; ++n) {
        const float y = p.update(1.0f, 0.0f);
        EXPECT_NEAR(toD(y), 10.0 * toD(kDt) * n, 1e-4) << "n=" << n;
    }
}

TEST(Pid, PAndIAddOnFirstStep) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 2.0f, 10.0f, 0.0f));
    // P = 2, I = 10 * dt * 1 = 0.0025
    EXPECT_NEAR(toD(p.update(1.0f, 0.0f)), 2.0025, 1e-5);
}

TEST(Pid, IntegratorIsClampedToLimit) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 10.0f, 0.0f, 0.2f));
    float y = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        y = p.update(1.0f, 0.0f);  // would reach 2.5 without a clamp
    }
    EXPECT_FLOAT_EQ(y, 0.2f);
}

TEST(Pid, ClampedIntegratorUnwindsQuickly) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 10.0f, 0.0f, 0.2f));
    for (int i = 0; i < 1000; ++i) {
        (void)p.update(1.0f, 0.0f);  // wind up against the limit
    }
    // Flip the error. Each step removes 0.0025, so ~80 steps reach zero.
    // Without the clamp this would take ~1000 steps. That gap is the feature.
    float y = 0.0f;
    for (int i = 0; i < 80; ++i) {
        y = p.update(0.0f, 1.0f);
    }
    EXPECT_NEAR(toD(y), 0.0, 1e-3);
}

TEST(Pid, ChangingKiMidRunDoesNotJumpTheIntegrator) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 10.0f, 0.0f, 100.0f));
    float before = 0.0f;
    for (int i = 0; i < 50; ++i) {
        before = p.update(1.0f, 0.0f);
    }
    ASSERT_TRUE(p.setGains(0.0f, 100.0f, 0.0f, kDt, 0.0f));
    const float after = p.update(1.0f, 0.0f);
    // Stored value is in output units, so only the new increment is added.
    EXPECT_NEAR(toD(after - before), 100.0 * toD(kDt), 1e-5);
}

// ---------- D ----------

TEST(Pid, NoDerivativeKickOnSetpointStep) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 1.0f));
    for (int i = 0; i < 20; ++i) {
        EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.0f) << "i=" << i;
    }
}

TEST(Pid, ConstantMeasurementGivesZeroD) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 1.0f));
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(p.update(0.0f, 5.0f * 0.0f), 0.0f);
    }
}

TEST(Pid, DOnRisingMeasurementIsNegativeAndSettlesToRate) {
    constexpr float kKd = 0.01f;
    constexpr float kCutoff = 100.0f;
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, kKd, 1.0f, kCutoff));

    // Measurement rises 0.1 per sample: raw D = -(kd/dt) * 0.1 = -4.
    const float rawD = -(kKd / kDt) * 0.1f;
    const float alpha = expectedAlpha(kCutoff, kDt);

    // First sample is filtered: alpha * rawD, not the full raw value.
    const float first = p.update(0.0f, 0.1f);
    EXPECT_LT(first, 0.0f);
    EXPECT_NEAR(toD(first), toD(alpha * rawD), 1e-4);

    // After many time constants it settles at the raw value.
    float y = first;
    for (int i = 2; i <= 200; ++i) {
        y = p.update(0.0f, 0.1f * static_cast<float>(i));
    }
    EXPECT_NEAR(toD(y), toD(rawD), 1e-3);
}

namespace {
// Peak |output| over the last 100 of 300 samples of +-1 alternating noise.
float peakOutputForNoise(fc::Pid& p) {
    float peak = 0.0f;
    for (int i = 0; i < 300; ++i) {
        const float m = (i % 2 == 0) ? 1.0f : -1.0f;
        const float y = p.update(0.0f, m);
        if (i >= 200 && (y > peak || -y > peak)) {
            peak = (y > 0.0f) ? y : -y;
        }
    }
    return peak;
}
}  // namespace

TEST(Pid, LowerDCutoffAttenuatesNoiseMore) {
    fc::Pid low;
    fc::Pid high;
    ASSERT_TRUE(configure(low, 0.0f, 0.0f, 0.01f, 1.0f, 50.0f));
    ASSERT_TRUE(configure(high, 0.0f, 0.0f, 0.01f, 1.0f, 1900.0f));
    const float peakLow = peakOutputForNoise(low);
    const float peakHigh = peakOutputForNoise(high);
    // Hand estimate: raw D swings +-80. Steady-state amplitude is
    // alpha*80/(2-alpha): ~3 at 50 Hz vs ~48 at 1900 Hz.
    EXPECT_GT(peakHigh, 10.0f);
    EXPECT_LT(peakLow, 0.1f * peakHigh);
}

// ---------- reset ----------

TEST(Pid, ResetClearsIntegratorAndFilterAndOutput) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1.0f, 10.0f, 0.01f, 1.0f));
    for (int i = 0; i < 100; ++i) {
        (void)p.update(1.0f, (i % 2 == 0) ? 1.0f : -1.0f);  // disturb everything
    }
    p.reset(0.0f, 0.0f);
    EXPECT_FLOAT_EQ(p.update(0.0f, 0.0f), 0.0f);
}

TEST(Pid, ResetSeedsMeasurementSoFirstDHasNoSpike) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 1.0f));
    p.reset(0.0f, 100.0f);
    EXPECT_FLOAT_EQ(p.update(0.0f, 100.0f), 0.0f);
}

// ---------- bad inputs ----------

TEST(Pid, NonFiniteInputReturnsPreviousOutput) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1.0f, 10.0f, 0.01f));
    const float good = p.update(1.0f, 0.0f);
    EXPECT_FLOAT_EQ(p.update(kNaN, 0.0f), good);
    EXPECT_FLOAT_EQ(p.update(1.0f, kNaN), good);
    EXPECT_FLOAT_EQ(p.update(kInf, 0.0f), good);
    EXPECT_FLOAT_EQ(p.update(1.0f, -kInf), good);
}

TEST(Pid, BadFirstInputAfterResetReturnsZero) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1.0f, 10.0f, 0.01f));
    EXPECT_FLOAT_EQ(p.update(kNaN, 0.0f), 0.0f);
}

TEST(Pid, BadSamplesDoNotChangeLaterBehavior) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 1.0f, 10.0f, 0.01f, 1.0f));
    ASSERT_TRUE(configure(b, 1.0f, 10.0f, 0.01f, 1.0f));
    for (int i = 0; i < 60; ++i) {
        if (i % 3 == 0) {
            (void)a.update(kNaN, 0.0f);   // only 'a' sees bad samples
            (void)a.update(1.0f, kInf);
        }
        const float m = 0.02f * static_cast<float>(i);
        EXPECT_FLOAT_EQ(a.update(1.0f, m), b.update(1.0f, m)) << "i=" << i;
    }
}

TEST(Pid, OutputStaysFiniteAfterBadSample) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1.0f, 0.0f, 0.01f));   // ki = 0 exposes 0*inf
    (void)p.update(1.0f, 0.0f);
    (void)p.update(1.0f, kInf);
    for (int i = 0; i < 100; ++i) {
        const float y = p.update(1.0f, 0.0f);
        EXPECT_EQ(y - y, 0.0f) << "non-finite output at i=" << i;
    }
}

// ---------- setters ----------

TEST(Pid, SetGainsRejectsInvalidValues) {
    fc::Pid p;
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, 0.0f, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, -kDt, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, kNaN, 0.0f));
    EXPECT_FALSE(p.setGains(-1.0f, 1.0f, 1.0f, kDt, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, -1.0f, 1.0f, kDt, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, -1.0f, kDt, 0.0f));
    EXPECT_FALSE(p.setGains(kNaN, 1.0f, 1.0f, kDt, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, kNaN, 1.0f, kDt, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, kNaN, kDt, 0.0f));
}

TEST(Pid, FailedSetGainsLeavesGainsUnchanged) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 1.0f, 10.0f, 0.01f, 1.0f));
    ASSERT_TRUE(configure(b, 1.0f, 10.0f, 0.01f, 1.0f));
    EXPECT_FALSE(a.setGains(5.0f, 5.0f, 5.0f, 0.0f, 0.0f));
    EXPECT_FALSE(a.setGains(kNaN, 5.0f, 5.0f, kDt, 0.0f));
    expectSameBehavior(a, b);
}

TEST(Pid, SetILimitRejectsInvalidValues) {
    fc::Pid p;
    EXPECT_FALSE(p.setILimit(0.0f));
    EXPECT_FALSE(p.setILimit(-1.0f));
    EXPECT_FALSE(p.setILimit(kNaN));
    EXPECT_FALSE(p.setILimit(kInf));
    EXPECT_TRUE(p.setILimit(0.5f));
}

TEST(Pid, FailedSetILimitLeavesLimitUnchanged) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 10.0f, 0.0f, 0.2f));
    EXPECT_FALSE(p.setILimit(-1.0f));
    EXPECT_FALSE(p.setILimit(kNaN));
    float y = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        y = p.update(1.0f, 0.0f);
    }
    EXPECT_FLOAT_EQ(y, 0.2f);
}

TEST(Pid, SetFilteredDRejectsInvalidCutoff) {
    fc::Pid p;
    EXPECT_FALSE(p.setFilteredD(0.0f, kDt));
    EXPECT_FALSE(p.setFilteredD(-10.0f, kDt));
    EXPECT_FALSE(p.setFilteredD(kNaN, kDt));
    EXPECT_FALSE(p.setFilteredD(2500.0f, kDt));  // above Nyquist (2000 Hz)
    EXPECT_TRUE(p.setFilteredD(100.0f, kDt));
}

TEST(Pid, FailedSetFilteredDLeavesFilterUnchanged) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 1.0f, 10.0f, 0.01f, 1.0f));
    ASSERT_TRUE(configure(b, 1.0f, 10.0f, 0.01f, 1.0f));
    EXPECT_FALSE(a.setFilteredD(0.0f, kDt));
    EXPECT_FALSE(a.setFilteredD(kNaN, kDt));
    expectSameBehavior(a, b);
}

// ---------- Feedforward ----------
// kff = 0.01 and dt = 1/4000 give Kff_ = kff/dt = 40.
// All other gains are 0, so the output is the FF term only.

namespace {
constexpr float kKff = 0.01f;
constexpr float kKffScaled = kKff / kDt;  // 40
constexpr float kFfCutoff = 100.0f;
}  // namespace

TEST(PidFF, ZeroKffAddsNothing) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1.0f, 0.0f, 0.0f));  // P only, kff = 0
    for (int i = 0; i < 100; ++i) {
        const float sp = 0.1f * static_cast<float>(i);  // moving setpoint
        EXPECT_FLOAT_EQ(p.update(sp, 0.0f), sp) << "i=" << i;
    }
}

TEST(PidFF, ConstantSetpointGivesZeroFF) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    p.reset(5.0f, 0.0f);  // setpoint already at 5, so no step
    // A self-assigned prevSetpoint_ would make this 40 * 5 = 200.
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(p.update(5.0f, 0.0f), 0.0f) << "i=" << i;
    }
}

TEST(PidFF, StepFirstSampleIsAlphaTimesKff) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    const float alpha = expectedAlpha(kFfCutoff, kDt);

    // Setpoint steps 0 -> 1 and stays. Raw FF is a one-sample pulse of 40.
    const float first = p.update(1.0f, 0.0f);
    EXPECT_NEAR(toD(first), toD(alpha * kKffScaled), 1e-4);

    // Then the filter decays: each sample is (1 - alpha) times the previous.
    const float second = p.update(1.0f, 0.0f);
    EXPECT_NEAR(toD(second), toD(first * (1.0f - alpha)), 1e-4);
}

TEST(PidFF, StepAreaIsPreserved) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    // A Pt1 has unity DC gain, so the total correction equals the raw
    // pulse (40), only spread over ~1/alpha samples.
    double sum = 0.0;
    for (int i = 0; i < 500; ++i) {
        sum += toD(p.update(1.0f, 0.0f));
    }
    EXPECT_NEAR(sum, toD(kKffScaled), 1e-2);
}

TEST(PidFF, RampSettlesToKffTimesRatePerSample) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    // Setpoint rises 0.1 per sample: raw FF = 40 * 0.1 = 4, constant.
    float y = 0.0f;
    for (int i = 1; i <= 300; ++i) {
        y = p.update(0.1f * static_cast<float>(i), 0.0f);
    }
    EXPECT_NEAR(toD(y), 4.0, 1e-2);
}

TEST(PidFF, ResetSeedsSetpointSoFirstFFHasNoSpike) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    p.reset(500.0f, 0.0f);
    EXPECT_FLOAT_EQ(p.update(500.0f, 0.0f), 0.0f);
}

TEST(PidFF, ResetClearsFilterState) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    (void)p.update(1.0f, 0.0f);  // leaves the FF filter nonzero
    p.reset(1.0f, 0.0f);
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.0f);
}

TEST(PidFF, NonFiniteSetpointDoesNotSpikeLaterFF) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 0.5f, 1.0f, 0.01f, 1.0f, 100.0f, kKff, kFfCutoff));
    ASSERT_TRUE(configure(b, 0.5f, 1.0f, 0.01f, 1.0f, 100.0f, kKff, kFfCutoff));
    for (int i = 0; i < 60; ++i) {
        if (i % 4 == 0) {
            (void)a.update(kNaN, 0.0f);  // only 'a' sees bad setpoints
            (void)a.update(kInf, 0.0f);
        }
        const float sp = 0.05f * static_cast<float>(i);
        EXPECT_FLOAT_EQ(a.update(sp, 0.0f), b.update(sp, 0.0f)) << "i=" << i;
    }
}

TEST(PidFF, SetFilteredFFRejectsInvalidCutoff) {
    fc::Pid p;
    EXPECT_FALSE(p.setFilteredFF(0.0f, kDt));
    EXPECT_FALSE(p.setFilteredFF(-10.0f, kDt));
    EXPECT_FALSE(p.setFilteredFF(kNaN, kDt));
    EXPECT_FALSE(p.setFilteredFF(2500.0f, kDt));  // above Nyquist
    EXPECT_TRUE(p.setFilteredFF(100.0f, kDt));
}

TEST(PidFF, FailedSetFilteredFFLeavesFilterUnchanged) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 1.0f, 10.0f, 0.01f, 1.0f, 100.0f, kKff, kFfCutoff));
    ASSERT_TRUE(configure(b, 1.0f, 10.0f, 0.01f, 1.0f, 100.0f, kKff, kFfCutoff));
    EXPECT_FALSE(a.setFilteredFF(0.0f, kDt));
    EXPECT_FALSE(a.setFilteredFF(kNaN, kDt));
    expectSameBehavior(a, b);
}

TEST(PidFF, SetGainsRejectsInvalidKff) {
    fc::Pid p;
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, kDt, -1.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, kDt, kNaN));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, kDt, kInf));
    EXPECT_TRUE(p.setGains(1.0f, 1.0f, 1.0f, kDt, 0.0f));
}

// ---------- Output limits (symmetric: [-limit, +limit]) ----------

TEST(PidOutput, ClampsToMaxAndMin) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 10.0f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);    // P = 10
    EXPECT_FLOAT_EQ(p.update(-1.0f, 0.0f), -0.5f);  // P = -10
}

TEST(PidOutput, InsideLimitsIsUnchanged) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.2f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.2f);
    EXPECT_FLOAT_EQ(p.update(-1.0f, 0.0f), -0.2f);
}

TEST(PidOutput, ValueExactlyOnLimitIsReturnedUnchanged) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.5f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);
}

TEST(PidOutput, ClampsTheSumNotTheIndividualTerms) {
    // ki = 4000 gives Ki_ = 1, so the integrator hits iLimit = 0.3 at once.
    // P = 0.4. Each term is below 0.5, but the sum 0.7 is not.
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.4f, 4000.0f, 0.0f, 0.3f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);
}

TEST(PidOutput, FeedforwardIsIncludedBeforeClamping) {
    // First FF sample after a unit step is alpha * 40 = ~5.4, far over 0.5.
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 0.0f, 1.0f, 100.0f, kKff, kFfCutoff));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);
}

TEST(PidOutput, HugeFiniteInputsStayWithinLimits) {
    // 1e10 * 1e30 overflows float to +inf; the clamp must still bound it.
    fc::Pid p;
    ASSERT_TRUE(configure(p, 1e10f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FLOAT_EQ(p.update(1e30f, 0.0f), 0.5f);
    EXPECT_FLOAT_EQ(p.update(-1e30f, 0.0f), -0.5f);
}

TEST(PidOutput, HeldOutputOnBadInputIsWithinLimits) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 10.0f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    ASSERT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);
    EXPECT_FLOAT_EQ(p.update(kNaN, 0.0f), 0.5f);  // held value is the clamped one
}

TEST(PidOutput, SetOutLimitRejectsInvalidValues) {
    fc::Pid p;
    EXPECT_FALSE(p.setOutLimit(0.0f));
    EXPECT_FALSE(p.setOutLimit(-0.5f));
    EXPECT_FALSE(p.setOutLimit(kNaN));
    EXPECT_FALSE(p.setOutLimit(kInf));
    EXPECT_TRUE(p.setOutLimit(0.5f));
}

TEST(PidOutput, FailedSetOutLimitLeavesLimitUnchanged) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 10.0f, 0.0f, 0.0f));
    ASSERT_TRUE(p.setOutLimit(0.5f));
    EXPECT_FALSE(p.setOutLimit(kNaN));
    EXPECT_FALSE(p.setOutLimit(-1.0f));
    EXPECT_FALSE(p.setOutLimit(kInf));
    EXPECT_FLOAT_EQ(p.update(1.0f, 0.0f), 0.5f);
    EXPECT_FLOAT_EQ(p.update(-1.0f, 0.0f), -0.5f);
}