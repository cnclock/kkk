#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"
#include "pid_controller.h"

class PIDController; // 前向声明

class MotorController {
public:
    MotorController(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, uint8_t limPin, TIM_HandleTypeDef* htim, uint32_t channel);
    void enable();
    void disable();
    void setDirection(int direction);
    void setFrequency(int frequency);
    void startHome();  //   启动回零
    bool isHomed();  //    判断是否回零
    void stop();
    void run(float distance1, float distance2, PIDController& pid);
    uint8_t getPwmPin() const;
    TIM_HandleTypeDef* getTimer() const;

    void setMaxFrequency(int frequency);    // 脉冲控制方法（每个电机独立状态）
    void togglePulse();

    // 处理TIM3中断
    static void handleTIM3Interrupt();

    // 处理TIM4中断
    static void handleTIM4Interrupt();
private:
    uint8_t limPin;
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t enPin;
    TIM_HandleTypeDef* htim;
    uint32_t timerChannel;
    bool isHoming;
    int maxFrequency;  // 最大频率 (Hz)
    int currentFrequency;
    uint32_t timerClockFreq;  // 定时器时钟频率 (Hz)
};

#endif // MOTOR_CONTROLLER_H