#pragma once
#include <array>

namespace fc{
    [[nodiscard]] bool mix(float throttle, float roll, float pitch, float yaw, std::array<float,4>& out);
}