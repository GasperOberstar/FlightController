#pragma once

namespace fc{
    [[nodiscard]] constexpr float clamp(float x, float lo, float hi){
        if(x>=lo && x<=hi)
            return x;
        else if(x>hi)
            return hi;
        return lo;
    }

    [[nodiscard]] constexpr bool isFinite(float x){ 
        return x - x == 0.0f; 
    }
}