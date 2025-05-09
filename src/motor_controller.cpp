#include "motor_controller.h"

// 声明外部电机实例（在main.cpp中定义）
extern MotorController motor1;
extern MotorController motor2;

MotorController::MotorController(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, uint8_t limPin, TIM_HandleTypeDef* htim, uint32_t channel)
    : pwmPin(pwmPin), dirPin(dirPin), enPin(enPin), limPin(limPin), htim(htim), timerChannel(channel) {
    pinMode(limPin, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    disable();
    
    isHoming = false;
    maxFrequency = 5000;  // 默认最大频率5kHz
    currentFrequency = 0;
    
    // 获取定时器时钟频率
    timerClockFreq = HAL_RCC_GetPCLK1Freq() * 2;  // 72MHz
}

void MotorController::setDirection(int direction) {
    digitalWrite(dirPin, direction);
}

void MotorController::enable() {
    digitalWrite(enPin, EN_LEVEL);
}

void MotorController::disable() {
    digitalWrite(enPin, UNEN_LEVEL);
    setFrequency(0);
}

void MotorController::stop() {
    setFrequency(0);
}

void MotorController::setFrequency(int frequency) {
    if (frequency > maxFrequency) frequency = maxFrequency;
    if (frequency <= 0) {
        // 停止定时器
        HAL_TIM_Base_Stop_IT(htim);
        currentFrequency = 0;
        return;
    }
    
    currentFrequency = frequency;
    
    // 计算自动重载值 (ARR)
    uint32_t period = timerClockFreq / (htim->Init.Prescaler + 1) / frequency - 1;
    
    // 更新定时器周期
    __HAL_TIM_SET_AUTORELOAD(htim, period);

    // 随机化初始计数值，避免多电机同步
    __HAL_TIM_SET_COUNTER(htim, rand() % (period + 1));
    
    // 启动定时器中断
    HAL_TIM_Base_Start_IT(htim);
}

void MotorController::startHome() {
    // 启动电机回零操作
    isHoming = true;
    setDirection(HOME_DIRECTION);
    setFrequency(HOME_FREQUENCY);
    enable();
}

bool MotorController::isHomed() {
    // 检查电机是否到达零点
    if (digitalRead(limPin) == LOW) {
        // 到达零点，停止电机
        disable();
        isHoming = false;
        return true;
    }
    return false;
}

void MotorController::run(float distance1, float distance2, PIDController& pid) {
    float avgDistance = min(distance1, distance2);
    float output = pid.compute(TARGET_DISTANCE, avgDistance);
    
    // 将PID输出映射到频率范围
    int frequency = map(constrain(output, 0, 100), 0, 100, 0, maxFrequency);
        
    // 设置电机方向
    setDirection(output >= 0 ? FORWARD : BACKWARD);

    // 设置频率
    setFrequency(abs(frequency));
    Serial2.print("output: ");
    Serial2.println(output);
}

uint8_t MotorController::getPwmPin() const {
    return pwmPin;
}

TIM_HandleTypeDef* MotorController::getTimer() const {
    return htim;
}

// 脉冲状态切换
void MotorController::togglePulse() {
    static bool pulseState = false;
    pulseState = !pulseState;
    digitalWrite(pwmPin, pulseState);
}

// 处理TIM3中断
void MotorController::handleTIM3Interrupt() {
    motor1.togglePulse();
}

// 处理TIM4中断
void MotorController::handleTIM4Interrupt() {
    motor2.togglePulse();
}
    
void MotorController::setMaxFrequency(int frequency) {
    if (frequency > 0) {
        maxFrequency = frequency;
    }
}