#pragma once

namespace fc{
    class Pt1{
    private:
        float k_=0.0f;
        float output_=0.0f;
    public:
        [[nodiscard]] bool init(float cutoffHz, float dt, float x);
        [[nodiscard]] float apply(float x);
        [[nodiscard]] bool setCutoff(float cutoffHz, float dt);
        void reset(float x);
        [[nodiscard]] float value() const { return output_; }
    };
}