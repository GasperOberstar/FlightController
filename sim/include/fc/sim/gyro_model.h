#pragma once

#include <vector>
#include <random>

namespace sim{
    class GyroModel{
    private:
        std::mt19937 rng_;
        std::normal_distribution<float> unitNormal_{0.0f, 1.0f};
        float noiseStd_=0.0f;
        std::vector<float> delayLine_;
        std::size_t head_=0;
        float lastOutput_=0.0f;
        float bias_=0.0f;

    public:
        [[nodiscard]]bool configure(float noiseStd, unsigned seed, std::size_t delaySample, float initialRate=0.0f, float bias);
        [[nodiscard]]float sample(float trueRate);
    };
}