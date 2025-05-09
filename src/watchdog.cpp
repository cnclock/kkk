// watchdog.cpp
#include "watchdog.h"
#include "stm32f1xx_hal.h"

IWDG_HandleTypeDef hiwdg;  // 添加全局句柄

Watchdog::Watchdog(uint32_t timeout)
    : timeout(timeout), lastFeedTime(0) {}

void Watchdog::start() {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = timeout / 2;  // 假设超时单位为ms
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
    lastFeedTime = millis();
}

void Watchdog::feed() {
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
    lastFeedTime = millis();
}