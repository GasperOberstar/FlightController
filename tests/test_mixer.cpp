#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <limits>

#include <fc/mixer.h>

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// Index -> motor: 0 rear-right, 1 front-right, 2 rear-left, 3 front-left.
constexpr std::size_t kRR = 0, kFR = 1, kRL = 2, kFL = 3;

std::array<float, 4> mixOk(float t, float r, float p, float y) {
    std::array<float, 4> out{};
    const bool ok = fc::mix(t, r, p, y, out);
    EXPECT_TRUE(ok);
    return out;
}

float sum(const std::array<float, 4>& a) { return a[0] + a[1] + a[2] + a[3]; }

}  // namespace

TEST(Mixer, ThrottleOnlyGivesEqualMotors) {
    const auto out = mixOk(0.5f, 0.0f, 0.0f, 0.0f);
    for (float m : out) EXPECT_FLOAT_EQ(m, 0.5f);
}

// ---------- physics: which motors speed up ----------

TEST(Mixer, PositiveRollSpeedsUpLeftMotors) {
    const auto out = mixOk(0.5f, 0.1f, 0.0f, 0.0f);
    EXPECT_GT(out[kRL], out[kRR]);
    EXPECT_GT(out[kRL], out[kFR]);
    EXPECT_GT(out[kFL], out[kRR]);
    EXPECT_GT(out[kFL], out[kFR]);
}

TEST(Mixer, PositivePitchSpeedsUpFrontMotors) {
    const auto out = mixOk(0.5f, 0.0f, 0.1f, 0.0f);
    EXPECT_GT(out[kFR], out[kRR]);
    EXPECT_GT(out[kFR], out[kRL]);
    EXPECT_GT(out[kFL], out[kRR]);
    EXPECT_GT(out[kFL], out[kRL]);
}

TEST(Mixer, PositiveYawSpeedsUpCcwMotors) {
    // CCW props: rear-right and front-left. Their reaction torque turns
    // the frame clockwise (nose right).
    const auto out = mixOk(0.5f, 0.0f, 0.0f, 0.1f);
    EXPECT_GT(out[kRR], out[kFR]);
    EXPECT_GT(out[kRR], out[kRL]);
    EXPECT_GT(out[kFL], out[kFR]);
    EXPECT_GT(out[kFL], out[kRL]);
}

// ---------- exact values ----------

TEST(Mixer, PureAxisCommandsGiveExactPattern) {
    const float t = 0.5f, c = 0.1f;
    const auto r = mixOk(t, c, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(r[kRR], t - c);
    EXPECT_FLOAT_EQ(r[kFR], t - c);
    EXPECT_FLOAT_EQ(r[kRL], t + c);
    EXPECT_FLOAT_EQ(r[kFL], t + c);

    const auto p = mixOk(t, 0.0f, c, 0.0f);
    EXPECT_FLOAT_EQ(p[kRR], t - c);
    EXPECT_FLOAT_EQ(p[kFR], t + c);
    EXPECT_FLOAT_EQ(p[kRL], t - c);
    EXPECT_FLOAT_EQ(p[kFL], t + c);

    const auto y = mixOk(t, 0.0f, 0.0f, c);
    EXPECT_FLOAT_EQ(y[kRR], t + c);
    EXPECT_FLOAT_EQ(y[kFR], t - c);
    EXPECT_FLOAT_EQ(y[kRL], t - c);
    EXPECT_FLOAT_EQ(y[kFL], t + c);
}

TEST(Mixer, AxisCommandsDoNotChangeTotalThrust) {
    EXPECT_NEAR(sum(mixOk(0.5f, 0.1f, 0.0f, 0.0f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.0f, 0.1f, 0.0f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.0f, 0.0f, 0.1f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.1f, -0.05f, 0.08f)), 2.0f, 1e-6);
}

TEST(Mixer, AxesAreIndependent) {
    // A combined command equals throttle plus the sum of the single-axis offsets.
    const auto a = mixOk(0.5f, 0.1f, 0.0f, 0.0f);
    const auto b = mixOk(0.5f, 0.0f, 0.07f, 0.0f);
    const auto c = mixOk(0.5f, 0.0f, 0.0f, 0.05f);
    const auto all = mixOk(0.5f, 0.1f, 0.07f, 0.05f);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(all[i], a[i] + b[i] + c[i] - 2.0f * 0.5f, 1e-6) << "i=" << i;
    }
}

TEST(Mixer, OppositeCommandsMirror) {
    const auto pos = mixOk(0.5f, 0.1f, 0.06f, 0.04f);
    const auto neg = mixOk(0.5f, -0.1f, -0.06f, -0.04f);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(pos[i] - 0.5f, -(neg[i] - 0.5f), 1e-6) << "i=" << i;
    }
}

// ---------- output range ----------

TEST(Mixer, OutputsAlwaysWithinZeroToOne) {
    const float throttles[] = {0.0f, 0.3f, 1.0f};
    const float cmds[] = {-2.0f, -0.5f, 0.0f, 0.5f, 2.0f};
    for (float t : throttles)
        for (float r : cmds)
            for (float p : cmds)
                for (float y : cmds) {
                    std::array<float, 4> out{};
                    ASSERT_TRUE(fc::mix(t, r, p, y, out));
                    for (float m : out) {
                        EXPECT_GE(m, 0.0f);
                        EXPECT_LE(m, 1.0f);
                    }
                }
}
TEST(Mixer, ShiftPreservesDifferencesNearTopSaturation) {
    const auto out = mixOk(0.9f, 0.3f, 0.0f, 0.0f);
    EXPECT_NEAR(out[kRL], 1.0f, 1e-6);
    EXPECT_NEAR(out[kFL], 1.0f, 1e-6);
    EXPECT_NEAR(out[kRR], 0.4f, 1e-6);
    EXPECT_NEAR(out[kFR], 0.4f, 1e-6);
}

TEST(Mixer, ShiftPreservesDifferencesNearBottomSaturation) {
    const auto out = mixOk(0.1f, 0.3f, 0.0f, 0.0f);
    EXPECT_NEAR(out[kRR], 0.0f, 1e-6);
    EXPECT_NEAR(out[kRL], 0.6f, 1e-6);
}

TEST(Mixer, LargeSpreadIsScaledNotClamped) {
    // d = {+0.8, -0.8, 0, 0}: roll 0.4 + pitch 0.4 gives FL +0.8, RR -0.8.
    const auto out = mixOk(0.5f, 0.4f, 0.4f, 0.0f);
    EXPECT_NEAR(out[kFL], 1.0f, 1e-6);
    EXPECT_NEAR(out[kRR], 0.0f, 1e-6);
    EXPECT_NEAR(out[kFR], 0.5f, 1e-6);
    EXPECT_NEAR(out[kRL], 0.5f, 1e-6);
}

// ---------- invalid input ----------

TEST(Mixer, NonFiniteInputReturnsFalse) {
    std::array<float, 4> out{};
    EXPECT_FALSE(fc::mix(kNaN, 0.0f, 0.0f, 0.0f, out));
    EXPECT_FALSE(fc::mix(0.5f, kNaN, 0.0f, 0.0f, out));
    EXPECT_FALSE(fc::mix(0.5f, 0.0f, kInf, 0.0f, out));
    EXPECT_FALSE(fc::mix(0.5f, 0.0f, 0.0f, -kInf, out));
}

TEST(Mixer, FailedMixLeavesOutputUntouched) {
    // Documents current behavior: on bad input 'out' is not written. The
    // caller must check the bool, or add a safe fallback (e.g. all zeros).
    std::array<float, 4> out{0.1f, 0.2f, 0.3f, 0.4f};
    EXPECT_FALSE(fc::mix(kNaN, 0.0f, 0.0f, 0.0f, out));
    EXPECT_FLOAT_EQ(out[0], 0.1f);
    EXPECT_FLOAT_EQ(out[3], 0.4f);
}