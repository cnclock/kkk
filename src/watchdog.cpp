#include "watchdog.h"

void Watchdog::setup(uint32_t timeout_ms) {
    // 配置看门狗
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // 设置预分频器（64）
    hiwdg.Init.Reload = (timeout_ms * 40) / 64; // 计算重装载值，40kHz 时钟

    // 初始化看门狗
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        // 如果初始化失败，可以添加错误处理
        Serial.println("看门狗初始化失败！");
        //while (1); // 停止程序
    }
}

void Watchdog::feed() {
    // 喂狗
    HAL_IWDG_Refresh(&hiwdg);
}