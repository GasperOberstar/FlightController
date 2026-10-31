#include<fc/pid.h>
#include<fc/math/utils.h>

namespace fc{
    bool Pid::setGains(float kp, float ki, float kd, float dt){
        if(!(dt>0.0f && kp>=0.0f && ki>=0.0f && kd>=0.0f))
            return false;
        Kp_=kp;
        Ki_=ki*dt;
        Kd_=kd/dt;
        return true;
    }

    bool Pid::setILimit(float limit){
        if(!(limit>0.0f))
            return false;
        iLimit_=limit;
        return true;
    }

    void Pid::reset(float measurement){
        prevMeasurement_=measurement;
        iTerm_=0.0f;
        dFilter_.reset(0.0f);
        prevOutput_=0.0f;
    }

    bool Pid::setFilteredD(float cutoffHz, float dt){
        return dFilter_.setCutoff(cutoffHz, dt);
    }

    float Pid::update(float setpoint, float measurement){
        if(isFinite(setpoint) && isFinite(measurement)){
            const float err=setpoint-measurement;
            const float P=Kp_*err;
            iTerm_=clamp(iTerm_+Ki_*err, -iLimit_, iLimit_);
            const float rawD=-Kd_*(measurement-prevMeasurement_);
            prevMeasurement_=measurement;
            prevOutput_=P+iTerm_+dFilter_.apply(rawD);
        }
        return prevOutput_;
    }
}