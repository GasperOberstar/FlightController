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
    const fc::MixResult res = fc::mix(t, r, p, y, out);
    EXPECT_TRUE(res.ok);
    return out;
}

fc::MixResult mixResult(float t, float r, float p, float y) {
    std::array<float, 4> out{};
    return fc::mix(t, r, p, y, out);
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
    EXPECT_NEAR(r[kRR], t - c, 1e-6);
    EXPECT_NEAR(r[kFR], t - c, 1e-6);
    EXPECT_NEAR(r[kRL], t + c, 1e-6);
    EXPECT_NEAR(r[kFL], t + c, 1e-6);

    const auto p = mixOk(t, 0.0f, c, 0.0f);
    EXPECT_NEAR(p[kRR], t - c, 1e-6);
    EXPECT_NEAR(p[kFR], t + c, 1e-6);
    EXPECT_NEAR(p[kRL], t - c, 1e-6);
    EXPECT_NEAR(p[kFL], t + c, 1e-6);

    const auto y = mixOk(t, 0.0f, 0.0f, c);
    EXPECT_NEAR(y[kRR], t + c, 1e-6);
    EXPECT_NEAR(y[kFR], t - c, 1e-6);
    EXPECT_NEAR(y[kRL], t - c, 1e-6);
    EXPECT_NEAR(y[kFL], t + c, 1e-6);
}

TEST(Mixer, AxisCommandsDoNotChangeTotalThrustWhenNotSaturated) {
    EXPECT_NEAR(sum(mixOk(0.5f, 0.1f, 0.0f, 0.0f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.0f, 0.1f, 0.0f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.0f, 0.0f, 0.1f)), 2.0f, 1e-6);
    EXPECT_NEAR(sum(mixOk(0.5f, 0.1f, -0.05f, 0.08f)), 2.0f, 1e-6);
}

TEST(Mixer, AxesAreIndependent) {
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

// ---------- saturation: shift and scale ----------

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
    EXPECT_NEAR(out[kFR], 0.0f, 1e-6);
    EXPECT_NEAR(out[kRL], 0.6f, 1e-6);
    EXPECT_NEAR(out[kFL], 0.6f, 1e-6);
}

TEST(Mixer, LargeSpreadIsScaledNotClamped) {
    // d = {-0.8 (RR), 0 (FR), 0 (RL), +0.8 (FL)}, spread 1.6, scale 0.625.
    const auto out = mixOk(0.5f, 0.4f, 0.4f, 0.0f);
    EXPECT_NEAR(out[kFL], 1.0f, 1e-6);
    EXPECT_NEAR(out[kRR], 0.0f, 1e-6);
    EXPECT_NEAR(out[kFR], 0.5f, 1e-6);
    EXPECT_NEAR(out[kRL], 0.5f, 1e-6);
}

TEST(Mixer, ThrottleIsNotScaledWhenSpreadIsTooLarge) {
    // Motors that carry no axis command keep the pilot's throttle.
    const auto out = mixOk(0.5f, 0.4f, 0.4f, 0.0f);
    EXPECT_NEAR(out[kFR], 0.5f, 1e-6);
    EXPECT_NEAR(out[kRL], 0.5f, 1e-6);
}

TEST(Mixer, ScalingPreservesAxisRatios) {
    // Roll 0.6 with yaw 0.3 at mid throttle: spread is 1.8, so it is scaled.
    // The ratio between the unscaled offsets must survive in the outputs.
    const float t = 0.5f;
    const auto small = mixOk(t, 0.2f, 0.0f, 0.1f);   // fits, scale 1
    const auto big = mixOk(t, 0.6f, 0.0f, 0.3f);     // 3x larger, scaled
    const fc::MixResult res = mixResult(t, 0.6f, 0.0f, 0.3f);
    ASSERT_TRUE(res.ok);
    ASSERT_LT(res.scale, 1.0f);
    // big offsets = 3 * small offsets * scale
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(big[i] - t, 3.0f * res.scale * (small[i] - t), 1e-5) << "i=" << i;
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
                    const fc::MixResult res = fc::mix(t, r, p, y, out);
                    ASSERT_TRUE(res.ok);
                    for (float m : out) {
                        EXPECT_GE(m, 0.0f);
                        EXPECT_LE(m, 1.0f);
                    }
                }
}

TEST(Mixer, DifferencesPreservedWheneverSpreadFits) {
    // Property: if no scaling happened, the differences between motors equal
    // the commanded (unclamped) differences, i.e. the shift is uniform.
    const float throttles[] = {0.0f, 0.2f, 0.5f, 0.8f, 1.0f};
    const float cmds[] = {-0.4f, -0.1f, 0.0f, 0.1f, 0.4f};
    for (float t : throttles)
        for (float r : cmds)
            for (float p : cmds)
                for (float y : cmds) {
                    std::array<float, 4> out{};
                    const fc::MixResult res = fc::mix(t, r, p, y, out);
                    ASSERT_TRUE(res.ok);
                    if (res.scale < 1.0f) continue;
                    // Roll difference between left and right motors.
                    // RL - RR = 2 * roll and FL - FR = 2 * roll when unclamped.
                    EXPECT_NEAR(out[kRL] - out[kRR], 2.0f * (r - y), 1e-5);
                    EXPECT_NEAR(out[kFL] - out[kFR], 2.0f * (r + y), 1e-5);
                }
}

// ---------- MixResult reporting ----------

TEST(MixerResult, NormalInputIsNotSaturated) {
    const fc::MixResult res = mixResult(0.5f, 0.1f, 0.05f, 0.02f);
    EXPECT_TRUE(res.ok);
    EXPECT_FALSE(res.saturated);
    EXPECT_FLOAT_EQ(res.scale, 1.0f);
    EXPECT_FLOAT_EQ(res.shift, 0.0f);
}

TEST(MixerResult, TopShiftIsReported) {
    const fc::MixResult res = mixResult(0.9f, 0.3f, 0.0f, 0.0f);
    EXPECT_TRUE(res.ok);
    EXPECT_TRUE(res.saturated);
    EXPECT_FLOAT_EQ(res.scale, 1.0f);
    EXPECT_NEAR(res.shift, -0.2f, 1e-6);
}

TEST(MixerResult, BottomShiftIsReported) {
    const fc::MixResult res = mixResult(0.1f, 0.3f, 0.0f, 0.0f);
    EXPECT_TRUE(res.ok);
    EXPECT_TRUE(res.saturated);
    EXPECT_NEAR(res.shift, 0.2f, 1e-6);
}

TEST(MixerResult, ScaleIsReportedWhenSpreadIsTooLarge) {
    const fc::MixResult res = mixResult(0.5f, 0.4f, 0.4f, 0.0f);
    EXPECT_TRUE(res.ok);
    EXPECT_TRUE(res.saturated);
    EXPECT_NEAR(res.scale, 0.625f, 1e-6);
    EXPECT_NEAR(res.shift, 0.0f, 1e-6);
}

TEST(MixerResult, ExactlyFittingCommandIsNotSaturated) {
    // Throttle 0.5 with roll 0.5: motors at exactly 0 and 1.
    const fc::MixResult res = mixResult(0.5f, 0.5f, 0.0f, 0.0f);
    EXPECT_TRUE(res.ok);
    EXPECT_FALSE(res.saturated);
}

// ---------- invalid input ----------

TEST(Mixer, NonFiniteInputIsRejected) {
    EXPECT_FALSE(mixResult(kNaN, 0.0f, 0.0f, 0.0f).ok);
    EXPECT_FALSE(mixResult(0.5f, kNaN, 0.0f, 0.0f).ok);
    EXPECT_FALSE(mixResult(0.5f, 0.0f, kInf, 0.0f).ok);
    EXPECT_FALSE(mixResult(0.5f, 0.0f, 0.0f, -kInf).ok);
}

TEST(Mixer, FailedMixLeavesOutputUntouched) {
    // Documents current behavior: on bad input 'out' is not written. The
    // caller must check .ok, or you add a safe fallback (e.g. all zeros).
    std::array<float, 4> out{0.1f, 0.2f, 0.3f, 0.4f};
    const fc::MixResult res = fc::mix(kNaN, 0.0f, 0.0f, 0.0f, out);
    EXPECT_FALSE(res.ok);
    EXPECT_FLOAT_EQ(out[0], 0.1f);
    EXPECT_FLOAT_EQ(out[3], 0.4f);
}

TEST(Mixer, FailedMixReportsNoSaturation) {
    const fc::MixResult res = mixResult(kNaN, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(res.ok);
    EXPECT_FALSE(res.saturated);
}