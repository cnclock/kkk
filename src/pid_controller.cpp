#include "pid_controller.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd, float integralLimit, float sampleTime)
    : kp(kp), ki(ki * sampleTime), kd(kd / sampleTime), integral(0), previousInput(0), integralLimit(integralLimit), sampleTime(sampleTime) {}

bool PIDController::isStable(float errorThreshold, unsigned long stableTimeMs) {
    // 获取当前误差
    float error = fabs(previousError); // 使用上一次输入值作为误差的参考

    // 判断误差是否小于阈值
    if (error < errorThreshold) {
        if (!stable) {
            stableStartTime = millis(); // 记录稳态开始时间
            stable = true;
        } else if (millis() - stableStartTime >= stableTimeMs) {
            return true; // 持续时间满足要求，认为稳态
        }
    } else {
        stable = false; // 如果误差超出范围，重置稳态状态
    }
    return false; // 未达到稳态
}

// PIDController类的compute函数，用于计算PID控制器的输出
float PIDController::compute(float setpoint, float input) {
    // 计算当前误差，即设定值与实际输入之间的差值
    float error = setpoint - input;
    // 保存当前误差到 previousError
    previousError = error;
    // 累加误差，用于积分项的计算
    integral += error;
    // 积分限幅，防止积分项过大导致的积分饱和现象
    if (integral > integralLimit) {
        integral = integralLimit; // 如果积分值超过上限，则将其限制在上限
    } else if (integral < -integralLimit) {
        integral = -integralLimit; // 如果积分值低于下限，则将其限制在下限
    }
    float derivative = -(input - previousInput); // 只对输入值进行微分
    float output = kp * error + ki * integral + kd * derivative;
    previousInput = input;

    // 返回 PID 控制器的输出
    return output;
}    

float PIDController::getPreviousInput() const {
    return previousInput; // 返回 previousInput 的值
}