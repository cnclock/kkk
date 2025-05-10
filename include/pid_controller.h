#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float integralLimit, float sampleTime); // 构造函数
    // 设置PID参数，包括比例系数kp，积分系数ki，微分系数kd，积分限制integralLimit，采样时间sampleTime

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