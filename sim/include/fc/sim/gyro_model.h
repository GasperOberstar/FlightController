#pragma once

#include <vector>
#include <random>

namespace sim{
    class GyroModel{
    private:
        std::mt19937 rng_;
        std::normal_distribution<float> unitNormal_{0.0f, 1.0f};
        float noiseStd_=0.0f;
        std::vector<float> delayLine_ = std::vector<float>(1, 0.0f);
        std::size_t head_=0;
        float lastOutput_=0.0f;
        float bias_=0.0f;

    public:
        [[nodiscard]]bool configure(float noiseStd, unsigned seed, std::size_t delaySample, float bias=0.0f,  float initialRate=0.0f);
        [[nodiscard]]float sample(float trueRate);
        void reset(float initalRate);
    };
}