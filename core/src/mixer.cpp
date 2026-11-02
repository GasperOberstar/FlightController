#include <fc/mixer.h>
#include <fc/math/utils.h>
#include <cstddef>
#include <algorithm>

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
        std::array<float, 4> rawOut;
        for(std::size_t i=0; i<out.size(); ++i){
            rawOut[i] = throttle+rool*kMixTable[i].roll+pitch*kMixTable[i].pitch+yaw*kMixTable[i].yaw;
        }
        const float lo=*std::min_element(rawOut.begin(), rawOut.end());
        const float hi=*std::max_element(rawOut.begin(), rawOut.end());
        const float spread=hi-lo;
                float shift;
        if(hi>1)
            shift=1-hi;
        else if(lo<0)
            shift=-lo;
        else
            shift=0;
        if(spread<=1){
            for(std::size_t i=0; i<out.size(); ++i){
                out[i]=clamp(rawOut[i]+shift+throttle, 0.0f, 1.0f);
            }
        }
        else{
            float scale=1/spread;
            std::array<float, 4> d;
            for(std::size_t i=0; i<out.size(); ++i){
                d[i]=rawOut[i]-throttle;
                out[i]=clamp(d[i]*scale+throttle+shift, 0.0f, 1.0f)
            }
        }
        return true;
    }
}