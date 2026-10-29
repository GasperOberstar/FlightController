#include <fc/pt1.h>
#include <fc/math/constants.h>

bool Pt1::init(float fc, float dt, float x){
    if(!setCutoff(fc, dt))
        return false;
    reset(x);
    return true;
}

void Pt1::reset(float x){
    output=x;
}

bool Pt1::setCutoff(float fc, float dt){
    if(fc<=0 || dt<=0)
        return false;
    float w=2.0f*Pi*fc*dt;
    k=(w) / (1.0f+w);
    return true;
}

float Pt1::apply(float x){
    output += k*(x-output);
    return output;
}