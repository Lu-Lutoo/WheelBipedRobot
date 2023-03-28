/**
/ @file cxx.cpp
/ @author 
/ @date 
*/

#include <PID.h>

void PID::Init(double k1, double k2, double k3, double k4,double t,double d){
    Kp = k1;
    Ki = k2;
    Kd = k3;
    I_out_Max = k4;
    TimeStep = t;
    target = d;
};

void PID::control(double newval, double &output){
    error = target - newval;

    P_out = Kp * error;
    I_out += Ki * error * TimeStep;
    D_out = Kd * (error - error_preval) / TimeStep;

    if(I_out > I_out_Max){
        I_out = I_out_Max;
    }
    else if(I_out < -I_out_Max){
        I_out = -I_out_Max;
    }

    error_preval = error;
    output = P_out + I_out + D_out;
};

void PID::set_target(double d){
    target = d;
}