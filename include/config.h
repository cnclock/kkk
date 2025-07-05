#pragma once
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <stdint.h>

// 系统状态枚举
enum SystemState {
    STATE_HOMING,
    STATE_READY,
    STATE_RUNNING,
    STATE_FAULT
};

// 硬件引脚配置
const u_int8_t PWM_PINS[4] = {PA0, PA6, PB6, PA8};
const u_int8_t DIR_PINS[4] = {PA1, PA7, PB7, PA9};
const u_int8_t  EN_PINS[4] = {PA4, PA5, PB5, PA10};

#define LIMIT_SW1_NEG        PC15     // 电机1负限位开关
#define LIMIT_SW2_NEG        PC14    // 电机2负限位开关
#define BUTTON_START         PC13   // 启动按钮

#define US1_TRIG             PB12    // 超声波1触发
#define US1_ECHO             PB13   // 超声波1回波--
#define US2_TRIG             PB14   // 超声波2触发
#define US2_ECHO             PB15   // 超声波2回波--
#define US3_TRIG             PB3    // 超声波3触发
#define US3_ECHO             PB4   // 超声波3回波--
#define US4_TRIG             PB8    // 超声波4触发
#define US4_ECHO             PB9    // 超声波4回波--

#define LED_DATA_PIN         PB10    // WS2812B数据引脚
#define LED_NUM              3         // WS2812B灯珠数量
#define BRUSH_PIN            PB11     // 继电器控制引脚

#define EN_LEVEL             HIGH   // 使能引脚电平
#define UNEN_LEVEL           LOW    // 非使能引脚电平
#define FORWARD              HIGH   // 正方向
#define BACKWARD             LOW    // 负方向
#define HOME_DIRECTION       BACKWARD  // 根据限位开关位置选择
#define HOME_FREQUENCY       6000    // 回原点速度
#define MOVE_SPEED           15       // 行走电机速度，以距离差值为比例单位

// 系统参数
const float TARGET_DISTANCE  = 8;   // 目标距离cm
const float START_DISTANCE   = 60;  // 起始距离cm
const float STABLE_DISTANCE  = 3;   // 稳态距离cm
const u_int16_t STABLE_TIME  = 1000; // 稳态时间ms
const u_int8_t STABLE_COUNT    = 3;   // 稳态计数，用于判断是否到达结束位置
const float SAFETY_DISTANCE  = 300;   // 安全阈值300cm
const u_int32_t MAX_STEPS     = 19000000;  // 最大步数限制
const u_int16_t MAX_FREQUENCY = 8000;  // 最大频率限制

const u_int32_t WATCHDOG_TIMEOUT = 256; // 看门狗超时时间(ms)

// EEPROM配置
#define EEPROM_START_ADDR    0x08080000  // STM32F103RC EEPROM起始地址
#define EEPROM_CRC_SEED      0xEDB88320  // CRC32多项式
