#pragma once
#include <Arduino.h>
#include "stm32f1xx_hal.h"  // 添加缺失的头文件


class Watchdog {
public:
    Watchdog(uint32_t timeout);
    void start();
    void feed();

private:
    uint32_t timeout;
    unsigned long lastFeedTime;
};