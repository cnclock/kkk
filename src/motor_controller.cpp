#include "motor_controller.h"
#include "MultiPwm.h"


extern MotorController motor1;  // 声明外部电机实例（在main.cpp中定义）
extern MotorController motor2;
extern MotorController motor3;
extern MotorController motor4;
extern MultiPwm pwm;            // 声明外部多路PWM实例（在main.cpp中定义）

MotorController::MotorController(uint8_t pwmPin, uint8_t dirPin, uint8_t enPin, ButtonHandler limPin, uint8_t pwmChannel)
    : pwmPin(pwmPin), dirPin(dirPin), enPin(enPin), limPin(limPin), pwmChannel(pwmChannel) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);

    digitalWrite(enPin, UNEN_LEVEL);
    maxFrequency = MAX_FREQUENCY;
    currentFrequency = 0;
    targetFrequency = 0;
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
    disable();
}

void MotorController::setFrequency(int frequency) {
    // 如果频率大于最大频率，则将频率设置为最大频率
    if (frequency > maxFrequency) frequency = maxFrequency;
    
    // 更新内部频率状态
    currentFrequency = frequency <= 0 ? 0 : frequency;
    targetFrequency = currentFrequency;
    
    // 设置PWM频率
    pwm.setFrequency(pwmChannel, currentFrequency);
    
    // 如果频率大于0，则启用电机
    if (currentFrequency > 0) {
        enable();
    }
}

void MotorController::startHome() {
    // 启动电机回零操作
    setDirection(HOME_DIRECTION);
    setFrequency(HOME_FREQUENCY); // 直接设置频率
    enable();
}

bool MotorController::isHomed() {
    // 检查电机是否到达零点
    if (this->limPin.isPressed()) {
        disable();
        return true;
    }
    return false;
}

void MotorController::run(float distance, PIDController& pid) {
    float output = pid.compute(TARGET_DISTANCE, distance);
    float limited = constrain(output, -100, 100);
    
    // 将PID输出映射到频率范围
    int frequency = map(abs(limited), 0, 100, 0, maxFrequency);
    
    // 设置电机方向
    setDirection(output <= 0 ? FORWARD : BACKWARD);
    
    // 设置频率（内部已包含enable逻辑）
    setFrequency(frequency);
}

uint8_t MotorController::getPwmPin() const {
    return pwmPin;
}

uint8_t MotorController::getPwmChannel() const {
    return pwmChannel;
}

void MotorController::setMaxFrequency(int frequency) {
    if (frequency > 0) {
        maxFrequency = frequency;
    }
}