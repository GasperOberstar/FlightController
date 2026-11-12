#include <iostream>
#include <fstream>

#include <fc/sim/plant_SingleAxis.h>
#include <fc/pid.h>
#include <fc/pt1.h>

int main(){
    const float inertia=1.0f;
    const float torqueGain=20.0f;
    const float motorFc=50.0f;
    const float gyroFc=150.0f;
    const float dFc=100.0f;
    const float ffFc=100.0f;
    const float dt=0.00025f;
    const float kp=0.2f;
    const float setPoint = 1.75f;

    sim::Plant plant;
    fc::Pid pid;
    fc::Pt1 gyroFilter;

    if(!(plant.configure(inertia, torqueGain, motorFc, dt))){std::cerr << "Plant config failed\n"; return 1;}
    if(!(pid.setGains(kp, 0.0f, 0.0f, dt, 0.0f))){std::cerr << "PID gains wrong\n"; return 1;}
    if(!(pid.setILimit(0.15f) && pid.setOutLimit(0.5f))){std::cerr << "PID limit config failed\n"; return 1;}
    if(!(pid.setFilteredD(dFc, dt) && pid.setFilteredFF(ffFc, dt))){std::cerr << "PID filter config failed\n"; return 1;}
    if (!gyroFilter.init(gyroFc, dt, 0.0f)) { std::cerr << "Gyro filter config failed\n"; return 1; }
    plant.reset(0.0f); pid.reset(0.0f, 0.0f);

    std::ofstream csv("sim_output/rate_loop.csv");
    if (!csv){ std::cerr << "cannot open CSV\n"; return 1; }
    csv << "t,setpoint,gyro,cmd,rate\n";
    const int steps=2000;
    for(int i=0; i<steps; ++i){
        const float t=static_cast<float>(i)*dt;
        const float sp=(t<0.05f) ? 0.0f : setPoint;
        const float gyro=gyroFilter.apply(plant.rate());
        const float cmd=pid.update(sp, gyro);
        (void)plant.step(cmd, 0.0f);
        csv << t << ',' << sp << ',' << gyro << ',' << cmd << ',' << plant.rate() << '\n';
    }
    csv.close();
    return 0;
}