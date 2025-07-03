#include "button_handler.h"

ButtonHandler::ButtonHandler(uint8_t pin)
    : pin(pin), lastState(HIGH), lastDebounceTime(0) {
    pinMode(pin, INPUT_PULLUP);
}

bool ButtonHandler::isPressed() {
    bool reading = digitalRead(pin);

    // 如果按键状态发生变化，更新去抖时间
    if (reading != lastState) {
        lastDebounceTime = millis();
        lastState = reading; // 始终更新 lastState
    }

    // 如果超过去抖延时，确认按键状态
    if ((millis() - lastDebounceTime) > 30) { // 去抖延时为 30ms
        if (lastState == LOW) { // 如果按键当前状态为 LOW，表示按下
            return true;
        }
    }

    return false;
}