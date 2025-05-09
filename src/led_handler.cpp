#include "led_handler.h"

LEDHandler::LEDHandler(uint8_t pin, uint16_t numPixels)
    : strip(numPixels, pin, NEO_GRB + NEO_KHZ800) {
    strip.begin();
    strip.show();
}

void LEDHandler::setColor(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
}

// 定义LEDHandler类的成员函数breatheEffect，用于实现LED灯的呼吸效果
// led_handler.cpp
void LEDHandler::breatheEffect(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
    static uint8_t brightness[256] = {0};  // 为每个LED创建独立状态
    static bool increasing[256] = {true};
    
    if (increasing[i]) {
        brightness[i]++;
        if (brightness[i] >= 255) increasing[i] = false;
    } else {
        brightness[i]--;
        if (brightness[i] <= 0) increasing[i] = true;
    }
    
    strip.setPixelColor(i, strip.Color(
        r * brightness[i] / 255, 
        g * brightness[i] / 255, 
        b * brightness[i] / 255
    ));
    strip.show();
}

// 定义一个名为LEDHandler的类的成员函数blinkEffect
// 该函数用于实现LED灯的闪烁效果
void LEDHandler::blinkEffect(uint8_t i, uint8_t r, uint8_t g, uint8_t b, uint16_t interval) {
    // 定义一个静态变量lastBlinkTime，用于记录上一次闪烁的时间
    static unsigned long lastBlinkTime = 0;
    // 定义一个静态变量isOn，用于记录当前LED灯的状态（开或关）
    static bool isOn = false;

    // 检查当前时间与上一次闪烁时间的差值是否大于指定的间隔时间interval
    if (millis() - lastBlinkTime > interval) {
        // 更新lastBlinkTime为当前时间
        lastBlinkTime = millis();
        // 切换LED灯的状态
        isOn = !isOn;
        // 如果LED灯当前状态为开，则设置LED灯的颜色为指定的r, g, b值
        if (isOn) {
            setColor(i, r, g, b);
        } else {
            // 如果LED灯当前状态为关，则设置LED灯的颜色为黑色（即关闭LED灯）
            setColor(i, 0, 0, 0);
        }
    }
}