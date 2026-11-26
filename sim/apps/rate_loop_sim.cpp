#include <iostream>
#include <fstream>

#include <fc/sim/plant_SingleAxis.h>
#include <fc/pid.h>
#include <fc/pt1.h>
#include <fc/sim/gyro_model.h>

int main(){
    const float inertia=1.0f;
    const float torqueGain=20.0f;
    const float motorFc=50.0f;
    const float gyroFc=150.0f;
    const float dFc=100.0f;
    const float ffFc=100.0f;
    const float dt=0.00025f;
    const float kp=1.0f;
    const float ki=0.25f;
    const float kd=0.015f;
    const float kff=0.05f;
    const float limit=0.15f;
    const float outLimit=1.0f;
    float setPoint=0.0f;
    const float noiseStd=0.02f;
    const unsigned seed=1u;
    const std::size_t delaySample=20;
    const float bias=0.02f;

    sim::Plant plant;
    sim::GyroModel gyroModel;
    fc::Pid pid;
    fc::Pt1 gyroFilter;

    if(!(plant.configure(inertia, torqueGain, motorFc, dt))){std::cerr << "Plant config failed\n"; return 1;}
    if(!(pid.setGains(kp, ki, kd, dt, kff))){std::cerr << "PID gains wrong\n"; return 1;}
    if(!(pid.setILimit(limit) && pid.setOutLimit(outLimit))){std::cerr << "PID limit config failed\n"; return 1;}
    if(!(pid.setFilteredD(dFc, dt) && pid.setFilteredFF(ffFc, dt))){std::cerr << "PID filter config failed\n"; return 1;}
    if(!gyroFilter.init(gyroFc, dt, 0.0f)) { std::cerr << "Gyro filter config failed\n"; return 1; }
    plant.reset(0.0f); pid.reset(0.0f, 0.0f);
    if (!gyroModel.configure(noiseStd, seed, delaySample, bias)) { std::cerr << "gyro model config failed\n"; return 1; }
    gyroModel.reset(0.0f);

    std::ofstream csv("sim_output/rate_loop.csv");
    if (!csv){ std::cerr << "cannot open CSV\n"; return 1; }
    csv << "t,setpoint,measured,gyro,cmd,rate\n";
    const int steps=2000;
    for(int i=0; i<steps; ++i){
        if(i<1000)
            setPoint+=(1.75f/1000.0f);
        else
            setPoint-=(1.75f/1000.0f);
        const float t=static_cast<float>(i)*dt;
        const float sp=(t<0.05f) ? 0.0f : setPoint;
        const float measured = gyroModel.sample(plant.rate());
        const float gyro=gyroFilter.apply(measured);
        const float cmd=pid.update(sp, gyro);
        (void)plant.step(cmd, 0.0f);
        csv << t << ',' << sp << ',' << measured << ',' << gyro << ',' << cmd << ',' << plant.rate() << '\n';
    }
    csv.close();
    return 0;
}