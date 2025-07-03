#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float integralLimit, float sampleTime); // 构造函数
    // 设置PID参数，包括比例系数kp，积分系数ki，微分系数kd，积分限制integralLimit，采样时间sampleTime

    float compute(float setpoint, float input);
    bool isStable(float errorThreshold, unsigned long stableTimeMs); // 新增函数声明
    float getPreviousInput() const; // 添加访问器函数

private:
    float kp;
    float ki;
    float kd;
    float integral;
    float previousInput; // 上一次输入值
    float integralLimit;
    float sampleTime;
    unsigned long stableStartTime; // 用于记录稳态开始时间
    bool stable; // 用于记录当前是否处于稳态
    float previousError; // 上一次误差


};    