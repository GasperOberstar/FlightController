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
               float iLimit = 100.0f, float dCutoffHz = 100.0f) {
    const bool ok = p.setGains(kp, ki, kd, kDt) && p.setILimit(iLimit) &&
                    p.setFilteredD(dCutoffHz, kDt);
    p.reset(0.0f);
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
    p.reset(3.0f);
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
    ASSERT_TRUE(p.setGains(0.0f, 100.0f, 0.0f, kDt));
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
    p.reset(0.0f);
    EXPECT_FLOAT_EQ(p.update(0.0f, 0.0f), 0.0f);
}

TEST(Pid, ResetSeedsMeasurementSoFirstDHasNoSpike) {
    fc::Pid p;
    ASSERT_TRUE(configure(p, 0.0f, 0.0f, 1.0f));
    p.reset(100.0f);
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
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, 0.0f));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, -kDt));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, 1.0f, kNaN));
    EXPECT_FALSE(p.setGains(-1.0f, 1.0f, 1.0f, kDt));
    EXPECT_FALSE(p.setGains(1.0f, -1.0f, 1.0f, kDt));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, -1.0f, kDt));
    EXPECT_FALSE(p.setGains(kNaN, 1.0f, 1.0f, kDt));
    EXPECT_FALSE(p.setGains(1.0f, kNaN, 1.0f, kDt));
    EXPECT_FALSE(p.setGains(1.0f, 1.0f, kNaN, kDt));
}

TEST(Pid, FailedSetGainsLeavesGainsUnchanged) {
    fc::Pid a;
    fc::Pid b;
    ASSERT_TRUE(configure(a, 1.0f, 10.0f, 0.01f, 1.0f));
    ASSERT_TRUE(configure(b, 1.0f, 10.0f, 0.01f, 1.0f));
    EXPECT_FALSE(a.setGains(5.0f, 5.0f, 5.0f, 0.0f));
    EXPECT_FALSE(a.setGains(kNaN, 5.0f, 5.0f, kDt));
    expectSameBehavior(a, b);
}

TEST(Pid, SetILimitRejectsInvalidValues) {
    fc::Pid p;
    EXPECT_FALSE(p.setILimit(0.0f));
    EXPECT_FALSE(p.setILimit(-1.0f));
    EXPECT_FALSE(p.setILimit(kNaN));
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