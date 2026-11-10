#pragma once
#include <fc/pt1.h>

namespace sim{
    class Plant {
    private:
        float omega_ = 0.0f;
        float inertia_ = 1.0f;
        float torqueGain_ = 0.0f;
        float dt_ = 0.0f;
        fc::Pt1 motor_;
    public:
    [[nodiscard]] bool configure(float inertia, float torqueGain,float motorFc, float dt);
    void reset(float omega);
    float step(float command, float disturbanceTorque);
    [[nodiscard]] float rate() const { return omega_; }
    };
}