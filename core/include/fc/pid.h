#pragma once
#include <fc/pt1.h>

namespace fc{
    class Pid{
    private:
        float Kp_=0.0f;
        float Ki_=0.0f; //Precomputed w/ dt
        float Kd_=0.0f; //Precomputed w/ dt
        float iTerm_=0.0f;
        float iLimit_=0.0f;
        float prevMeasurement_=0.0f;
        float prevOutput_=0.0f;
        Pt1 dFilter_;
    public:
        [[nodiscard]]float update(float setpoint, float measurement);
        void reset(float measurement);
        [[nodiscard]] bool setGains(float kp, float ki, float kd, float dt);
        [[nodiscard]] bool setILimit(float limit);
        [[nodiscard]] bool setFilteredD(float cutoffHz, float dt);
    };
}