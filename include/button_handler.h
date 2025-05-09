#pragma once
#include <Arduino.h>

class ButtonHandler {
public:
    ButtonHandler(uint8_t pin);
    bool isPressed();

private:
    uint8_t pin;
    bool lastState;
    unsigned long lastDebounceTime;
};