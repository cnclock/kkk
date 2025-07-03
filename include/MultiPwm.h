#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

class MultiPwm {
public:
    MultiPwm(const uint8_t* pins, size_t num);
    ~MultiPwm();
    void begin(uint32_t freq = 1000, float duty = 50.0f);
    void setFrequency(uint8_t ch, uint32_t freq);
    void setDuty(uint8_t ch, float duty);

private:
    struct TimerMap {
        uint8_t pin;
        TIM_TypeDef* timer;
        uint8_t channel;
    };
    HardwareTimer** timers;
    const uint8_t* pwmPins;
    size_t pwmNum;
    static const TimerMap timerMap[4];
    HardwareTimer* getTimerByPin(uint8_t pin);
    uint8_t getTimerChannelByPin(uint8_t pin);
};