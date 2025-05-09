#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float integralLimit, float sampleTime);
    float compute(float setpoint, float input);

private:
    float kp;
    float ki;
    float kd;
    float integral;
    float previousInput;
    float integralLimit;
    float sampleTime;
};    