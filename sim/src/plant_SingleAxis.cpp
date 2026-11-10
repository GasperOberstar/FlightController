#include <fc/sim/plant_SingleAxis.h>
#include <fc/pid.h>
#include <fc/math/utils.h>

namespace sim{
    bool Plant::configure(float inertia, float torqueGain, float motorFc, float dt){
        if(!(fc::isFinite(inertia) && fc::isFinite(torqueGain) && fc::isFinite(motorFc) && fc::isFinite(dt) && 
             inertia > 0.0f && torqueGain > 0.0f && motorFc > 0.0f && dt > 0.0f))
            return false;
        fc::Pt1 candidate = motor_;
        if (!candidate.setCutoff(motorFc, dt))
        return false;

        inertia_ = inertia;
        torqueGain_ = torqueGain;
        dt_ = dt;
        motor_ = candidate;
        return true;
    }

    void Plant::reset(float omega){
        omega_=omega;
        motor_.reset(0.0f);
    }

    float Plant::step(float command, float disturbanceTorque){
        if (!(fc::isFinite(command) && fc::isFinite(disturbanceTorque)))
        return omega_;

        const float torque = motor_.apply(torqueGain_ * command) + disturbanceTorque;
        omega_ += (torque / inertia_) * dt_;
        return omega_;
    }
}