#pragma once
#include <array>

namespace fc{
    struct MixResult{
        bool ok=false;
        bool saturated=false;
        float scale=1.0f;
        float shift=0.0f; 
    };
    [[nodiscard]] MixResult mix(float throttle, float roll, float pitch, float yaw, std::array<float,4>& out);
}