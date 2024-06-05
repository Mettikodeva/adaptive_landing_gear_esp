#pragma once

class PID{
public:
    PID();
    PID(float kp, float ki, float kd);
    PID(float kp, float ki, float kd, float int_sat);
    ~PID();
    void setKp(int kp);
    void setKi(int ki);
    void setKd(int kd);
    void setIntSat(int int_sat);
    
    float update(float error);
    float update(float setpoint, float current);
    float update(float setpint, float current, float dt);

private:
    float _kp=0, _ki=0, _kd=0, _dt=0;
    float _integral=0;
    float _integral_saturation=10;
    float _prev_error=0;
};
