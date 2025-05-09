#include "button_handler.h"

ButtonHandler::ButtonHandler(uint8_t pin)
    : pin(pin), lastState(HIGH), lastDebounceTime(0) {
    pinMode(pin, INPUT_PULLUP);
}

bool ButtonHandler::isPressed() {
    bool reading = digitalRead(pin);
    
    // 检测状态变化
    if (reading != lastState) {
        lastDebounceTime = millis();
    }
    
    // 防抖处理
    if ((millis() - lastDebounceTime) > 50) {
        if (reading != lastState) {
            lastState = reading;
            // 返回按下事件（下降沿）
            return (lastState == LOW);
        }
    }
    
    return false;
}