#include <fc/pt1.h>
#include <fc/math/constants.h>

namespace fc{
    bool Pt1::init(float cutoffHz, float dt, float x){
        if(!setCutoff(cutoffHz, dt))
            return false;
        reset(x);
        return true;
    }

    void Pt1::reset(float x){
        output_=x;
    }

    bool Pt1::setCutoff(float cutoffHz, float dt){
        if(!(cutoffHz>0.0f && dt>0.0f) || cutoffHz>=1.0f/(2.0f*dt))
            return false;
        float w=2.0f*kPi*cutoffHz*dt;
        k_=(w) / (1.0f+w);
        return true;
    }

    float Pt1::apply(float x){
        output_ += k_*(x-output_);
        return output_;
    }
}