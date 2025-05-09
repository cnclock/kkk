// watchdog.h
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include "stm32f1xx_hal.h"      // 添加 HAL 库头文件
#include "stm32f1xx_hal_iwdg.h" // 添加 IWDG 头文件

class Watchdog {
public:
    Watchdog(uint32_t timeout); // 构造函数，设置看门狗超时时间

    // 初始化看门狗
    void start();

    // 喂狗
    void feed();

private:
    IWDG_HandleTypeDef hiwdg; // 看门狗句柄
    uint32_t timeout;
    uint32_t lastFeedTime;      // 上次喂狗时间;
};
#endif // WATCHDOG_H