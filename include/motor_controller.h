#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"
#include "pid_controller.h"
#include "button_handler.h"

class PIDController; // 前向声明

class MotorController {
public:
    MotorController(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, ButtonHandler limPin, uint8_t pwmChannel);
    void enable();
    void disable();
    void setDirection(int direction);
    void setFrequency(int frequency);
    TIM_HandleTypeDef* getTimer() const;
    void startHome();                       //   启动回零
    bool isHomed();                         //   判断是否回零
    void stop();
    void run(float distance,PIDController& pid);
    uint8_t getPwmPin() const;
    uint8_t getPwmChannel() const;          // 获取PWM通道
    void setMaxFrequency(int frequency);    // 脉冲控制方法（每个电机独立状态）

    // 处理TIM3中断
    static void handleTIM3Interrupt();

    // 处理TIM4中断

private:
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t enPin;
    uint8_t pwmChannel; // 新增：用于标识PWM通道
    TIM_HandleTypeDef* htim;
    uint32_t timerChannel;
    //bool isHoming;
    int maxFrequency;  // 最大频率 (Hz)
    int currentFrequency;
    int targetFrequency; // 目标频率 (用于软启动)
    unsigned long lastRampTime; // 上次更新频率的时间
    bool isSoftStarting; // 是否处于软启动状态
    uint32_t timerClockFreq;  // 定时器时钟频率 (Hz)
    ButtonHandler limPin;
};

#endif // MOTOR_CONTROLLER_H