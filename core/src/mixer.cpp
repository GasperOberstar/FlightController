#include <fc/mixer.h>
#include <fc/math/utils.h>
#include <cstddef>

namespace fc{
    /* rool -> right side up
       pitch -> nose up
       yaw -> nose right
    */
    namespace{struct MixEntry{float roll, pitch, yaw;};}
    /*Index -> motor: 0 = rear-right (CCW), 1 = front-right (CW),
                      2 = rear-left (CW),   3 = front-left (CCW).*/
    constexpr std::array<MixEntry, 4> kMixTable {{ {-1.0f, -1.0f,  1.0f},  // 0 rear-right,  CCW
                                                   {-1.0f,  1.0f, -1.0f},  // 1 front-right, CW
                                                   { 1.0f, -1.0f, -1.0f},  // 2 rear-left,   CW
                                                   { 1.0f,  1.0f,  1.0f}, // 3 front-left,  CCW
    }};
    
    bool mix(float throttle, float roll, float pitch, float yaw, std::array<float,4>& out){
        if(!(isFinite(throttle) && isFinite(roll) && isFinite(pitch) && isFinite(yaw)))
            return false;
        for(std::size_t i=0; i<out.size(); ++i){
            out[i] = clamp(throttle+roll*kMixTable[i].roll+pitch*kMixTable[i].pitch+yaw*kMixTable[i].yaw, 0, 1);
        }
        return true;
    }
}