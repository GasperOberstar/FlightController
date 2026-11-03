#include <fc/mixer.h>
#include <fc/math/utils.h>
#include <cstddef>
#include <algorithm>
#include <array>

namespace fc{
    /* roll -> right side down
       pitch -> nose up
       yaw -> nose right
    */
    namespace{struct MixEntry{float roll, pitch, yaw;};}
    /*Index -> motor: 0 = rear-right (CCW), 1 = front-right (CW),
                      2 = rear-left (CW),   3 = front-left (CCW).*/
    constexpr std::array<MixEntry, 4> kMixTable {{ {-1.0f, -1.0f,  1.0f},  // 0 rear-right,  CCW
                                                   {-1.0f,  1.0f, -1.0f},  // 1 front-right, CW
                                                   { 1.0f, -1.0f, -1.0f},  // 2 rear-left,   CW
                                                   { 1.0f,  1.0f,  1.0f},  // 3 front-left,  CCW
    }};
    
    MixResult mix(float throttle, float roll, float pitch, float yaw, std::array<float,4>& out){
        if(!(isFinite(throttle) && isFinite(roll) && isFinite(pitch) && isFinite(yaw)))
            return MixResult{};
        std::array<float, 4> rawOut{{0.0f,0.0f,0.0f,0.0f}};
        for(std::size_t i=0; i<out.size(); ++i){
            rawOut[i] = throttle+roll*kMixTable[i].roll+pitch*kMixTable[i].pitch+yaw*kMixTable[i].yaw;
        }
        const float spread=*std::max_element(rawOut.begin(), rawOut.end())-*std::min_element(rawOut.begin(), rawOut.end());
        float shift=0.0f;
        float scale=1.0f;
        if(spread<=1.0f){
            const float lo=*std::min_element(rawOut.begin(), rawOut.end());
            const float hi=*std::max_element(rawOut.begin(), rawOut.end());
            shift = (hi > 1.0f) ? 1.0f - hi : (lo < 0.0f) ? -lo : 0.0f;
            for(std::size_t i=0; i<out.size(); ++i){
                out[i]=clamp(rawOut[i]+shift, 0.0f, 1.0f);
            }
        }
        else{
            scale=1.0f/spread;
            std::array<float, 4> d{{0.0f,0.0f,0.0f,0.0f}};
            for(std::size_t i=0; i<out.size(); ++i){
                d[i]=rawOut[i]-throttle;
            }
            const float lo=*std::min_element(d.begin(), d.end())*scale+throttle;
            const float hi=*std::max_element(d.begin(), d.end())*scale+throttle;
            shift = (hi > 1.0f) ? 1.0f - hi : (lo < 0.0f) ? -lo : 0.0f;
            for(std::size_t i=0; i<out.size(); ++i){
                out[i]=clamp(d[i]*scale+throttle+shift, 0.0f, 1.0f);
            }
        }
        MixResult r;
        r.ok=true;
        r.scale=scale;
        r.shift=shift;
        r.saturated=(scale<1.0f) || (shift!=0.0f);
        return r;
    }
}