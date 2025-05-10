#include "motor_controller.h"

// 声明外部电机实例（在main.cpp中定义）
extern MotorController motor1;
extern MotorController motor2;

MotorController::MotorController(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, uint8_t limPin, TIM_HandleTypeDef* htim, uint32_t channel)
    : pwmPin(pwmPin), dirPin(dirPin), enPin(enPin), limPin(limPin), htim(htim), timerChannel(channel) {
    //pinMode(limPin, INPUT_PULLUP);
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    //disable();

    isHoming = false;
    maxFrequency = MAX_FREQUENCY;
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
        HAL_TIM_PWM_Stop(htim, timerChannel);
        currentFrequency = 0;
        return;
    }

    currentFrequency = frequency;

    // 获取定时器时钟频率
    uint32_t timerClockFreq = HAL_RCC_GetPCLK1Freq() * 2;  // 假设 APB1 预分频器大于 1
    uint32_t timerFreq = timerClockFreq / (htim->Init.Prescaler + 1);

    // 计算周期和比较值
    uint32_t period = timerFreq / frequency - 1;
    uint32_t compareValue = period / 2; // 50%占空比

    // 设置定时器参数
    __HAL_TIM_SET_AUTORELOAD(htim, period);
    __HAL_TIM_SET_COMPARE(htim, timerChannel, compareValue); 

    // 启动 PWM
    HAL_TIM_PWM_Start(htim, timerChannel);
    enable();

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
    //if (digitalRead(limPin) == LOW) {
    if (this->limPin.isPressed()) {
        disable();
        isHoming = false;
        return true;
    }
    return false;
}

void MotorController::run(float distance1, float distance2, PIDController& pid) {
    //float minDistance = min(distance1, distance2);
    float output = pid.compute(TARGET_DISTANCE, min(distance1, distance2));
    
    // 将PID输出映射到频率范围
    int frequency = map(constrain(output, -100, 100), 0, 100, 0, maxFrequency);
        
    // 设置电机方向
    setDirection(output <= 0 ? FORWARD : BACKWARD);

    // 设置频率
    setFrequency(abs(frequency));
    Serial2.print("output: ");
    Serial2.println(output);
    Serial2.print("frequency: ");
    Serial2.println(frequency);}

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