#pragma once
#include "config.h"
class ButtonHandler {
public:
    ButtonHandler(uint8_t pin);
    bool isPressed();
    uint8_t getPin() const { return pin; } // 添加获取引脚号的方法

private:
    uint8_t pin;
    bool lastState;
    unsigned long lastDebounceTime;
};