#include "pid.h"

PID::PID() : PID(0.1, 0.0, 0.0, 0.0){}
PID::PID(float kp, float ki, float kd) : PID(kp, ki, kd, 0.0){}
PID::PID(float kp, float ki, float kd, float int_sat){
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral_saturation = int_sat;
}

void PID::setKp(int kp){
    _kp = kp;
}
void PID::setKi(int ki){
    _ki = ki;
}

void PID::setKd(int kd){
    _kd = kd;
}

void PID::setIntSat(int int_sat){
    _integral_saturation = int_sat;
}

float PID::update(float error){
    if (error > 100){
        error = 100;
        }
    else if (error < -100){
        error = -100;
    }
    float out = _kp * error;
    _integral += error;
    if(_integral>_integral_saturation)
        _integral = _integral_saturation;   
    if(_integral<-_integral_saturation)
        _integral = -_integral_saturation;
        
    if (!_dt)
        out += _ki * _integral +_kd * _prev_error;
    else
        out += _ki * _integral +_kd * _prev_error*_dt;
    _prev_error = error;

    return out;
}

float PID::update(float setpoint, float current){
    return PID::update(setpoint - current);
}

float PID::update(float setppoint, float current, float dt){
    _dt = dt;
    return PID::update(setppoint - current);
}