#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <fc/sim/plant_SingleAxis.h>

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

constexpr float kDt = 1.0f / 4000.0f;   // 4 kHz, period in seconds
constexpr float kInertia = 2.0f;        // kg*m^2
constexpr float kGain = 4.0f;           // N*m at command = 1
constexpr float kMotorFc = 100.0f;      // Hz

// Steady-state rate increment per step at command 1, once the motor settles:
// kGain * 1 / kInertia * dt = 2 * dt = 5e-4.
constexpr double kSteadyInc = 4.0 / 2.0 / 4000.0;

double toD(float x) { return static_cast<double>(x); }

// Motor filter pole q = 1 - alpha, computed independently of Pt1.
// alpha = dt / (tau + dt), tau = 1 / (2*pi*fc).
double motorAlpha() {
    const double pi = 3.14159265358979323846;
    const double dt = 1.0 / 4000.0;
    const double tau = 1.0 / (2.0 * pi * 100.0);
    return dt / (tau + dt);
}

// Returns false if configure rejected; wrap in ASSERT_TRUE.
bool configured(sim::Plant& p) {
    const bool ok = p.configure(kInertia, kGain, kMotorFc, kDt);
    p.reset(0.0f);
    return ok;
}

}  // namespace

// ---------- basic physics ----------

TEST(Plant, ZeroCommandKeepsInitialRate) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    p.reset(1.5f);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_FLOAT_EQ(p.step(0.0f, 0.0f), 1.5f) << "i=" << i;
    }
}

TEST(Plant, DisturbanceAcceleratesAsTorqueOverInertia) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    // No command, so the motor outputs 0. Explicit Euler is exact here:
    // omega = omega0 + (d / J) * n * dt = 0.5 * 1000 / 4000 = 0.125.
    for (int i = 0; i < 1000; ++i) {
        (void)p.step(0.0f, 1.0f);
    }
    EXPECT_NEAR(toD(p.rate()), 0.125, 1e-4);
}

TEST(Plant, PositiveCommandGivesPositiveRate) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    for (int i = 0; i < 50; ++i) {
        (void)p.step(1.0f, 0.0f);
    }
    EXPECT_GT(p.rate(), 0.0f);
}

TEST(Plant, NegativeCommandGivesNegativeRate) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    for (int i = 0; i < 50; ++i) {
        (void)p.step(-1.0f, 0.0f);
    }
    EXPECT_LT(p.rate(), 0.0f);
}

TEST(Plant, CommandAndDisturbanceAddUp) {
    // After the motor settles, torque = kGain*cmd + disturbance.
    // Command 1 gives 4 N*m, disturbance -4 cancels it: rate stops changing.
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    for (int i = 0; i < 300; ++i) {
        (void)p.step(1.0f, -4.0f);
    }
    const float before = p.rate();
    for (int i = 0; i < 100; ++i) {
        (void)p.step(1.0f, -4.0f);
    }
    EXPECT_NEAR(toD(p.rate()), toD(before), 1e-5);
}

// ---------- motor lag ----------

TEST(Plant, SteadyCommandGivesConstantAcceleration) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    // 300 steps is ~44 time constants, so the lag has fully settled.
    for (int i = 0; i < 300; ++i) {
        (void)p.step(1.0f, 0.0f);
    }
    const float a = p.rate();
    const float b = p.step(1.0f, 0.0f);
    EXPECT_NEAR(toD(b - a), kSteadyInc, 1e-6);
}

TEST(Plant, FirstStepIsAlphaTimesSteadyIncrement) {
    // Motor starts at 0, so the first torque is alpha * (gain * command).
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    const float y = p.step(1.0f, 0.0f);
    EXPECT_NEAR(toD(y), motorAlpha() * kSteadyInc, 1e-7);
}

TEST(Plant, RateMatchesClosedFormWithMotorLag) {
    // torque_k = G*c*(1 - q^k), q = 1 - alpha. Summing the increments:
    // omega_n = (G*c*dt/J) * (n - q*(1 - q^n)/alpha).
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    const double a = motorAlpha();
    const double q = 1.0 - a;
    for (int n = 1; n <= 100; ++n) {
        (void)p.step(1.0f, 0.0f);
        const double expected =
            kSteadyInc * (n - q * (1.0 - std::pow(q, n)) / a);
        EXPECT_NEAR(toD(p.rate()), expected, 1e-5) << "n=" << n;
    }
}

TEST(Plant, TorqueIsAboutSixtyThreePercentAfterOneTimeConstant) {
    // tau/dt = ~6.37 samples, so n = 6. Same discretization gap as the
    // Pt1 test (0.583 vs 0.632), so the same 0.07 tolerance is justified.
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    float previous = 0.0f;
    float incAt6 = 0.0f;
    for (int n = 1; n <= 6; ++n) {
        const float y = p.step(1.0f, 0.0f);
        incAt6 = y - previous;
        previous = y;
    }
    EXPECT_NEAR(toD(incAt6) / kSteadyInc, 0.632, 0.07);
}

// ---------- reset ----------

TEST(Plant, ResetSetsRate) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    p.reset(3.0f);
    EXPECT_FLOAT_EQ(p.rate(), 3.0f);
}

TEST(Plant, ResetClearsMotorLagState) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    for (int i = 0; i < 100; ++i) {
        (void)p.step(1.0f, 0.0f);  // leaves the motor filter nonzero
    }
    p.reset(0.0f);
    // With leftover lag state the rate would move even with zero command.
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(p.step(0.0f, 0.0f), 0.0f) << "i=" << i;
    }
}

// ---------- configure validation ----------

TEST(Plant, ConfigureRejectsInvalidInertia) {
    sim::Plant p;
    EXPECT_FALSE(p.configure(0.0f, kGain, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(-1.0f, kGain, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(kNaN, kGain, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(kInf, kGain, kMotorFc, kDt));
}

TEST(Plant, ConfigureRejectsInvalidTorqueGain) {
    sim::Plant p;
    EXPECT_FALSE(p.configure(kInertia, 0.0f, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(kInertia, -1.0f, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(kInertia, kNaN, kMotorFc, kDt));
    EXPECT_FALSE(p.configure(kInertia, kInf, kMotorFc, kDt));
}

TEST(Plant, ConfigureRejectsInvalidMotorCutoff) {
    sim::Plant p;
    EXPECT_FALSE(p.configure(kInertia, kGain, 0.0f, kDt));
    EXPECT_FALSE(p.configure(kInertia, kGain, -5.0f, kDt));
    EXPECT_FALSE(p.configure(kInertia, kGain, kNaN, kDt));
    EXPECT_FALSE(p.configure(kInertia, kGain, kInf, kDt));
    EXPECT_FALSE(p.configure(kInertia, kGain, 2500.0f, kDt));  // above Nyquist
}

TEST(Plant, ConfigureRejectsInvalidDt) {
    sim::Plant p;
    EXPECT_FALSE(p.configure(kInertia, kGain, kMotorFc, 0.0f));
    EXPECT_FALSE(p.configure(kInertia, kGain, kMotorFc, -kDt));
    EXPECT_FALSE(p.configure(kInertia, kGain, kMotorFc, kNaN));
    EXPECT_FALSE(p.configure(kInertia, kGain, kMotorFc, kInf));
}

TEST(Plant, ConfigureAcceptsValidValues) {
    sim::Plant p;
    EXPECT_TRUE(p.configure(kInertia, kGain, kMotorFc, kDt));
}

TEST(Plant, FailedConfigureLeavesPlantUnchanged) {
    sim::Plant a;
    sim::Plant b;
    ASSERT_TRUE(configured(a));
    ASSERT_TRUE(configured(b));
    // The last call fails only on the cutoff, after the other values are valid.
    EXPECT_FALSE(a.configure(10.0f, 10.0f, 2500.0f, kDt));
    EXPECT_FALSE(a.configure(0.0f, 1.0f, 50.0f, kDt));
    EXPECT_FALSE(a.configure(kNaN, 1.0f, 50.0f, kDt));
    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(a.step(1.0f, 0.1f), b.step(1.0f, 0.1f)) << "i=" << i;
    }
}

// ---------- bad step input ----------

TEST(Plant, NonFiniteStepInputIsIgnored) {
    sim::Plant p;
    ASSERT_TRUE(configured(p));
    for (int i = 0; i < 20; ++i) {
        (void)p.step(1.0f, 0.0f);
    }
    const float r = p.rate();
    EXPECT_FLOAT_EQ(p.step(kNaN, 0.0f), r);
    EXPECT_FLOAT_EQ(p.step(1.0f, kNaN), r);
    EXPECT_FLOAT_EQ(p.step(kInf, 0.0f), r);
    EXPECT_FLOAT_EQ(p.step(1.0f, -kInf), r);
}

TEST(Plant, BadStepsDoNotChangeLaterBehavior) {
    sim::Plant a;
    sim::Plant b;
    ASSERT_TRUE(configured(a));
    ASSERT_TRUE(configured(b));
    for (int i = 0; i < 60; ++i) {
        if (i % 5 == 0) {
            (void)a.step(kNaN, 0.0f);  // only 'a' sees bad inputs
            (void)a.step(1.0f, kInf);
        }
        EXPECT_FLOAT_EQ(a.step(0.5f, 0.1f), b.step(0.5f, 0.1f)) << "i=" << i;
    }
}

// ---------- unconfigured ----------

TEST(Plant, UnconfiguredPlantIsInert) {
    // Documents current behavior: dt = 0 and gain = 0, so nothing moves.
    sim::Plant p;
    for (int i = 0; i < 10; ++i) {
        EXPECT_FLOAT_EQ(p.step(1.0f, 1.0f), 0.0f);
    }
}