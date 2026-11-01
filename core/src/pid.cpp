#include<fc/pid.h>
#include<fc/math/utils.h>

namespace fc{
    bool Pid::setGains(float kp, float ki, float kd, float dt, float kff){
        if(!(isFinite(dt) && isFinite(kp) && isFinite(ki) && isFinite(kd) && isFinite(kff) && 
        dt>0.0f && kp>=0.0f && ki>=0.0f && kd>=0.0f && kff>=0.0f))
            return false;
        Kp_=kp;
        Ki_=ki*dt;
        Kd_=kd/dt;
        Kff_=kff/dt;
        return true;
    }

    bool Pid::setILimit(float limit){
        if(!(limit>0.0f && isFinite(limit)))
            return false;
        iLimit_=limit;
        return true;
    }

    bool Pid::setOutLimit(float limit){
        if(!(limit>0.0f && isFinite(limit)))
            return false;
        outLimit_=limit;
        return true;
    }

    void Pid::reset(float setpoint,float measurement){
        prevMeasurement_=measurement;
        iTerm_=0.0f;
        dFilter_.reset(0.0f);
        ffFilter_.reset(0.0f);
        prevOutput_=0.0f;
        prevSetpoint_=setpoint;
    }

    bool Pid::setFilteredD(float cutoffHz, float dt){
        return dFilter_.setCutoff(cutoffHz, dt);
    }

    bool Pid::setFilteredFF(float cutoffHz, float dt){
        return ffFilter_.setCutoff(cutoffHz, dt);
    }

    float Pid::update(float setpoint, float measurement){
        if(isFinite(setpoint) && isFinite(measurement)){
            const float err=setpoint-measurement;
            const float P=Kp_*err;
            iTerm_=clamp(iTerm_+Ki_*err, -iLimit_, iLimit_);
            const float rawD=-Kd_*(measurement-prevMeasurement_);
            const float rawFF=Kff_*(setpoint-prevSetpoint_);
            prevMeasurement_=measurement;
            prevSetpoint_=setpoint;
            prevOutput_=clamp(P+iTerm_+dFilter_.apply(rawD)+ffFilter_.apply(rawFF), -outLimit_, outLimit_);
        }
        return prevOutput_;
    }
}