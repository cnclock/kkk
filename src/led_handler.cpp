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
void LEDHandler::breatheEffect(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
    // 静态变量brightness用于存储当前亮度值，初始值为0
    static uint8_t brightness = 0;
    // 静态变量increasing用于指示亮度是否正在增加，初始值为true
    static bool increasing = true;

    // 如果当前亮度正在增加
    if (increasing) {
        // 亮度值增加1
        brightness++;
        // 如果亮度值达到最大值255，则改变方向，开始减少亮度
        if (brightness >= 255) {
            increasing = false;
        }
    } else {
        // 亮度值减少1
        brightness--;
        // 如果亮度值降到最小值0，则改变方向，开始增加亮度
        if (brightness <= 0) {
            increasing = true;
        }
    }

    // 遍历LED灯带的所有像素点
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
        // 设置当前像素点的颜色，颜色值根据当前亮度进行线性缩放
        strip.setPixelColor(i, strip.Color(r * brightness / 255, g * brightness / 255, b * brightness / 255));
    }
    // 更新LED灯带显示
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