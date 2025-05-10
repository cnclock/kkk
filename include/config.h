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
/*
PA0 - 
PA1 - 
PA2 -  USART2_TX         //已使用
PA3 -  USART2_RX         //已使用
PA4 -  MOTOR2_DIR_PIN    //已使用
PA5 -  MOTOR1_DIR_PIN    //已使用
PA6 -  TIM3_CH1          //已使用
PA7 -  TIM3_CH2
PA8 -  TIM1_CH1
PA9 -  UART1_TX
PA10 - UART1_RX
PA11 - USB_DM
PA12 - USB_DP
PA13 - SWDIO
PA14 - SWCLK
PA15 - 

PB0 - TIM3_CH3 LIMIT_SW1_NEG     //已使用
PB1 - LED_DATA_PIN     //已使用
PB2 - I2C1_SCL
PB3 - I2C1_SDA
PB4 - TIM3_CH4
PB5 - TIM3_CH2 LIMIT_SW2_NEG     //已使用
PB6 - I2C1_SCL
PB7 - I2C1_SDA
PB8 - TIM4_CH3
PB9 - TIM4_CH4
PB10 - TIM2_CH3
PB11 - TIM2_CH4
PB12 - TIM1_CH1
PB13 - TIM1_CH2
PB14 - TIM1_CH3
PB15 - TIM1_CH4

PC0 - LCD1602 RS    //已使用
PC1 - LCD1602 EN    //已使用
PC2 - LCD1602 D4    //已使用
PC3 - LCD1602 D5    //已使用
PC4 - LCD1602 D6    //已使用
PC5 - LCD1602 D7    //已使用
PC6 -
PC7 -
PC8 -
PC9 -
PC10 -
PC11 -
PC12 -
PC13 - BUTTON_START //已使用
PC14 - MOTOR1_EN    //已使用
PC15 - MOTOR2_EN    //已使用



*/


// 硬件引脚配置
#define MOTOR1_PWM_PIN       PA6    //TIM3 的 PWM 输出通道 CH1：PA6、PB4，CH2：PA7、PB5，CH3：PB0，CH4：PB1
#define MOTOR1_DIR_PIN       PA5
#define MOTOR1_EN_PIN        PC14
#define MOTOR2_PWM_PIN       PB6    //TIM4 的 PWM 输出通道 
#define MOTOR2_DIR_PIN       PA4
#define MOTOR2_EN_PIN        PC15

#define LIMIT_SW1_NEG        PA9     // 电机1负限位开关
#define LIMIT_SW2_NEG        PA10    // 电机2负限位开关
#define BUTTON_START         PC13   // 启动按钮

#define US1_TRIG             PB9    // 超声波1触发
#define US1_ECHO             PB11   // 超声波1回波--
#define US2_TRIG             PB13   // 超声波2触发
#define US2_ECHO             PB15   // 超声波2回波--
#define US3_TRIG             PB8    // 超声波3触发
#define US3_ECHO             PB10   // 超声波3回波--
#define US4_TRIG             PB12    // 超声波4触发
#define US4_ECHO             PB14    // 超声波4回波--

#define LED_DATA_PIN         PB1    // WS2812B数据引脚
#define LED_NUM              3      // WS2812B灯珠数量

// LCD1602并口配置 (RS, EN, D4-D7)
const uint8_t lcdPins[6] = {PC0, PC1, PC2, PC3, PC4, PC5};


#define EN_LEVEL             HIGH   // 使能引脚电平
#define UNEN_LEVEL           LOW    // 非使能引脚电平
#define FORWARD              HIGH   // 正方向
#define BACKWARD             LOW    // 负方向
#define HOME_DIRECTION       BACKWARD  // 根据限位开关位置选择
#define HOME_FREQUENCY       4000    // 回原点速度



// 系统参数
const float TARGET_DISTANCE  = 50.0;  // 目标距离10cm
const float SAFETY_DISTANCE  = 80.0;  // 安全阈值50cm
const uint16_t MAX_STEPS     = 1000;  // 最大步数限制
const uint16_t MAX_FREQUENCY = 9000;  // 最大频率限制

const uint32_t WATCHDOG_TIMEOUT = 256; // 看门狗超时时间(ms)

// EEPROM配置
#define EEPROM_START_ADDR    0x08080000  // STM32F103RC EEPROM起始地址
#define EEPROM_CRC_SEED      0xEDB88320  // CRC32多项式

