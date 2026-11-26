#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <fc/sim/gyro_model.h>

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

double toD(float x) { return static_cast<double>(x); }

// Zero noise, zero bias: a pure delay line.
bool pureDelay(sim::GyroModel& g, std::size_t delay, float initial = 0.0f) {
    return g.configure(0.0f, 1u, delay, 0.0f, initial);
}

}  // namespace

// ---------- delay line ----------

TEST(GyroModel, DelayShiftsInputByExactlyNSamples) {
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 3));
    // Inputs 1..6 -> outputs 0,0,0,1,2,3 (pre-fill 0, then input k-3).
    const float expected[] = {0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f};
    for (int k = 1; k <= 6; ++k) {
        EXPECT_FLOAT_EQ(g.sample(static_cast<float>(k)), expected[k - 1]) << "k=" << k;
    }
}

TEST(GyroModel, DelayZeroIsPassThrough) {
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 0));
    for (int k = 1; k <= 20; ++k) {
        const float v = 0.5f * static_cast<float>(k);
        EXPECT_FLOAT_EQ(g.sample(v), v) << "k=" << k;
    }
}

TEST(GyroModel, LongDelayWrapsCorrectly) {
    // Many wraps of the ring buffer: output k must equal input k - N.
    constexpr std::size_t N = 7;
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, N));
    for (int k = 1; k <= 200; ++k) {
        const float out = g.sample(static_cast<float>(k));
        const float expected = (k > static_cast<int>(N)) ? static_cast<float>(k - static_cast<int>(N)) : 0.0f;
        EXPECT_FLOAT_EQ(out, expected) << "k=" << k;
    }
}

TEST(GyroModel, ConfigureInitialRatePrefillsDelayLine) {
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 3, 5.0f));
    // First N outputs are the initial rate, not a fake jump from zero.
    for (int k = 0; k < 3; ++k) {
        EXPECT_FLOAT_EQ(g.sample(9.0f), 5.0f) << "k=" << k;
    }
    EXPECT_FLOAT_EQ(g.sample(9.0f), 9.0f);
}

TEST(GyroModel, ResetRefillsDelayLine) {
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 3));
    for (int k = 1; k <= 10; ++k) (void)g.sample(static_cast<float>(k));
    g.reset(2.0f);
    for (int k = 0; k < 3; ++k) {
        EXPECT_FLOAT_EQ(g.sample(7.0f), 2.0f) << "k=" << k;
    }
    EXPECT_FLOAT_EQ(g.sample(7.0f), 7.0f);
}

TEST(GyroModel, ReconfigureWithSmallerDelayWorks) {
    // A stale head_ or an old buffer size would index out of range or
    // give the wrong delay.
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 10));
    for (int k = 1; k <= 25; ++k) (void)g.sample(static_cast<float>(k));
    ASSERT_TRUE(pureDelay(g, 1));
    EXPECT_FLOAT_EQ(g.sample(4.0f), 0.0f);
    EXPECT_FLOAT_EQ(g.sample(5.0f), 4.0f);
    EXPECT_FLOAT_EQ(g.sample(6.0f), 5.0f);
}

// ---------- bias ----------

TEST(GyroModel, BiasIsAddedToOutput) {
    sim::GyroModel g;
    ASSERT_TRUE(g.configure(0.0f, 1u, 0, 0.25f, 0.0f));
    EXPECT_FLOAT_EQ(g.sample(1.0f), 1.25f);
    EXPECT_FLOAT_EQ(g.sample(-2.0f), -1.75f);
}

TEST(GyroModel, BiasIsNotStoredInDelayLine) {
    // With delay 2 the output is input(k-2) + bias, applied once.
    sim::GyroModel g;
    ASSERT_TRUE(g.configure(0.0f, 1u, 2, 0.5f, 0.0f));
    (void)g.sample(1.0f);
    (void)g.sample(2.0f);
    EXPECT_FLOAT_EQ(g.sample(3.0f), 1.5f);  // 1 + 0.5, not 1 + 1.0
}

// ---------- noise ----------

TEST(GyroModel, ZeroNoiseIsExact) {
    sim::GyroModel g;
    ASSERT_TRUE(pureDelay(g, 0));
    EXPECT_EQ(g.sample(1.2345f), 1.2345f);
}

TEST(GyroModel, SameSeedGivesSameSequence) {
    sim::GyroModel a, b;
    ASSERT_TRUE(a.configure(0.5f, 42u, 2));
    ASSERT_TRUE(b.configure(0.5f, 42u, 2));
    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(a.sample(1.0f), b.sample(1.0f)) << "i=" << i;
    }
}

TEST(GyroModel, ReconfigureWithSameSeedReproduces) {
    // Needs unitNormal_.reset() in configure (cached second value).
    sim::GyroModel g;
    ASSERT_TRUE(g.configure(0.5f, 7u, 0));
    std::vector<float> first;
    for (int i = 0; i < 11; ++i) first.push_back(g.sample(0.0f));  // odd count
    ASSERT_TRUE(g.configure(0.5f, 7u, 0));
    for (int i = 0; i < 11; ++i) {
        EXPECT_EQ(g.sample(0.0f), first[static_cast<std::size_t>(i)]) << "i=" << i;
    }
}

TEST(GyroModel, DifferentSeedsGiveDifferentSequences) {
    sim::GyroModel a, b;
    ASSERT_TRUE(a.configure(0.5f, 1u, 0));
    ASSERT_TRUE(b.configure(0.5f, 2u, 0));
    int differing = 0;
    for (int i = 0; i < 100; ++i) {
        if (a.sample(0.0f) != b.sample(0.0f)) ++differing;
    }
    EXPECT_GT(differing, 90);
}

TEST(GyroModel, NoiseHasExpectedMeanAndStd) {
    constexpr int N = 100000;
    constexpr float kStd = 0.5f;
    constexpr float kTrue = 2.0f;
    sim::GyroModel g;
    ASSERT_TRUE(g.configure(kStd, 123u, 0));

    double sum = 0.0, sumSq = 0.0;
    for (int i = 0; i < N; ++i) {
        const double e = toD(g.sample(kTrue)) - toD(kTrue);
        sum += e;
        sumSq += e * e;
    }
    const double mean = sum / N;
    const double var = sumSq / N - mean * mean;
    // Mean error has std = kStd/sqrt(N) = 0.0016; 5 sigma tolerance.
    EXPECT_NEAR(mean, 0.0, 5.0 * toD(kStd) / std::sqrt(static_cast<double>(N)));
    // Sample std has std ~ kStd/sqrt(2N) = 0.0011; 0.01 is a wide margin.
    EXPECT_NEAR(std::sqrt(var), toD(kStd), 0.01);
}

TEST(GyroModel, NoiseIsNotStoredInDelayLine) {
    // Noise is added when reading. If it were stored, the noise on the
    // value read at k would depend on the noise drawn at k-N. Compare a
    // delayed model against a delay-0 model with the same seed: the
    // noise sequence is identical, so out_delayed(k) - in(k-N) must equal
    // out_nodelay(k) - in(k).
    sim::GyroModel d, n;
    ASSERT_TRUE(d.configure(0.3f, 9u, 3));
    ASSERT_TRUE(n.configure(0.3f, 9u, 0));
    for (int k = 1; k <= 50; ++k) {
        const float in = static_cast<float>(k);
        const float od = d.sample(in);
        const float on = n.sample(in);
        const float inDelayed = (k > 3) ? static_cast<float>(k - 3) : 0.0f;
        EXPECT_NEAR(toD(od - inDelayed), toD(on - in), 1e-4) << "k=" << k;
    }
}

// ---------- invalid input ----------

TEST(GyroModel, ConfigureRejectsInvalidValues) {
    sim::GyroModel g;
    EXPECT_FALSE(g.configure(-0.1f, 1u, 0));
    EXPECT_FALSE(g.configure(kNaN, 1u, 0));
    EXPECT_FALSE(g.configure(kInf, 1u, 0));
    EXPECT_FALSE(g.configure(0.1f, 1u, 0, kNaN));
    EXPECT_FALSE(g.configure(0.1f, 1u, 0, kInf));
    EXPECT_FALSE(g.configure(0.1f, 1u, 0, 0.0f, kNaN));  // needs isFinite(initialRate)
    EXPECT_FALSE(g.configure(0.1f, 1u, 100000));         // absurd delay
    EXPECT_TRUE(g.configure(0.1f, 1u, 50));
}

TEST(GyroModel, FailedConfigureLeavesModelUnchanged) {
    sim::GyroModel a, b;
    ASSERT_TRUE(a.configure(0.2f, 5u, 3, 0.1f, 1.0f));
    ASSERT_TRUE(b.configure(0.2f, 5u, 3, 0.1f, 1.0f));
    EXPECT_FALSE(a.configure(-1.0f, 99u, 8));
    EXPECT_FALSE(a.configure(0.2f, 99u, 100000));
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(a.sample(static_cast<float>(i)), b.sample(static_cast<float>(i)))
            << "i=" << i;
    }
}

TEST(GyroModel, NonFiniteInputIsIgnored) {
    sim::GyroModel a, b;
    ASSERT_TRUE(a.configure(0.2f, 5u, 3));
    ASSERT_TRUE(b.configure(0.2f, 5u, 3));
    for (int i = 0; i < 60; ++i) {
        // Bad inputs go to 'a' only. They must not store NaN, advance the
        // buffer, or consume noise, so the two models stay in lockstep.
        const float before = a.sample(static_cast<float>(i));
        EXPECT_EQ(before, b.sample(static_cast<float>(i)));
        if (i % 4 == 0) {
            const float held = a.sample(kNaN);
            EXPECT_EQ(held, before);          // returns the last output
            (void)a.sample(kInf);
        }
    }
}

TEST(GyroModel, OutputStaysFiniteAfterBadInput) {
    sim::GyroModel g;
    ASSERT_TRUE(g.configure(0.1f, 3u, 4));
    (void)g.sample(kNaN);
    (void)g.sample(kInf);
    for (int i = 0; i < 100; ++i) {
        const float y = g.sample(1.0f);
        EXPECT_EQ(y - y, 0.0f) << "non-finite output at i=" << i;
    }
}

TEST(GyroModel, UnconfiguredModelDoesNotCrash) {
    // Header default is a 1-slot buffer, so this must be safe (pass-through).
    sim::GyroModel g;
    EXPECT_FLOAT_EQ(g.sample(3.0f), 3.0f);
}