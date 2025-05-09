#include "config.h"
#include "eeprom_manager.h"
#include "motor_controller.h"
#include "ultrasonic.h"
#include "button_handler.h"
#include "led_handler.h"
#include "lcd_display.h"
#include "watchdog.h"
#include "pid_controller.h"
#include "stm32f1xx_hal.h"  // 添加HAL库头文件
#include <Ultrasonic.h>

// 定时器句柄
TIM_HandleTypeDef htim3;  // 假设使用TIM3
TIM_HandleTypeDef htim4;  // 假设使用TIM4

// 实例化对象
MotorController motor1(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_EN_PIN, LIMIT_SW1_NEG, &htim3, TIM_CHANNEL_1);
MotorController motor2(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_EN_PIN, LIMIT_SW2_NEG, &htim4, TIM_CHANNEL_1);
Ultrasonic ultrasonic1(US1_TRIG, US1_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(US2_TRIG, US2_ECHO);    // An ultrasonic sensor HC-04
ButtonHandler startButton(BUTTON_START);
LEDHandler led(LED_DATA_PIN, 3);
LCDDisplay lcd(lcdPins);
Watchdog watchdog(WATCHDOG_TIMEOUT);
//设置PID初始参数
PIDController pid1(1.0, 0.1, 0.01, 100.0, 0.1); 
PIDController pid2(1.0, 0.1, 0.01, 100.0, 0.1); 
EEPROMManager eeprom;

SystemState currentState = STATE_HOMING;
uint16_t steps = 0;
void Error_Handler(void); // 函数声明
void home();
void run();
void fault();

// 定时器初始化函数（基于频率控制）
void TIM3_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;      // 72MHz / 72 = 1MHz时钟
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;       // 初始频率1kHz (1MHz/1000)
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 启动TIM3
    HAL_TIM_Base_Start_IT(&htim3); // 启用定时器中断
}

void TIM4_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72 - 1;      // 72MHz / 72 = 1MHz时钟
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 1000 - 1;       // 初始频率1kHz
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 启动TIM4
    HAL_TIM_Base_Start_IT(&htim4); // 启用定时器中断
}

// 添加TIM3和TIM4的中断服务函数
extern "C" {
    void TIM3_IRQHandler(void) {
        MotorController::handleTIM3Interrupt();
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE); // 清除中断标志
    }

    void TIM4_IRQHandler(void) {
        MotorController::handleTIM4Interrupt();
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE); // 清除中断标志
    }
}

void home() {
    Serial2.println("电机回零... ");
    motor1.startHome();
    motor2.startHome();
    // 等待两个电机都回零
    while (!motor1.isHomed() || !motor2.isHomed()) { 
        // 添加看门狗喂狗操作
        watchdog.feed();
        delay(10);
    }
    currentState = STATE_READY;
    Serial2.println("就绪，请按开始键... ");
}

void run() {
    float distance1 = ultrasonic1.read();
    float distance2 = ultrasonic2.read();
    Serial2.println("运行中...");
    Serial2.print("distance1: ");
    Serial2.println(distance1);
    Serial2.print("distance2: ");
    Serial2.println(distance2);  
    motor1.run(distance1, distance2, pid1);
    motor2.run(distance1, distance2, pid2); // 修复：启用motor2控制

    // 添加看门狗喂狗操作
    watchdog.feed();
}

void fault() {
    motor1.disable();
    motor2.disable();
    while (!startButton.isPressed()) {
        led.blinkEffect(0, 64, 0, 0, 500);
        // 添加看门狗喂狗操作
        watchdog.feed();
        delay(10);
    }
}

void setup() {
    currentState = STATE_HOMING;
    Serial.begin(9600);
    Serial2.begin(9600);
    led.setColor(0, 64, 0, 0);
    Serial2.println("输出调试信息");
    Serial2.print("System Clock Frequency: ");
    Serial2.print(HAL_RCC_GetHCLKFreq());
    Serial2.println(" Hz");
    Serial2.println("输出调试信息");
    
    // 初始化看门狗
    Serial2.print("初始化看门狗... ");
    watchdog.start();
    Serial2.println("OK.");
    
    // 初始化电机控制器
    Serial2.print("初始化电机控制器... ");
    TIM3_Init(); // 初始化PWM输出
    TIM4_Init(); // 初始化PWM输出
    Serial2.println("OK.");
    
    // 初始化NVIC中断优先级
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    
    home();
}

void loop() {
    watchdog.feed();
    
    if (Serial2.available() > 0) {
        int newFrequency = Serial.parseInt();
        if (newFrequency >= 0 && newFrequency <= 10000) { // 可根据实际情况调整最大频率
            motor1.setMaxFrequency(newFrequency);
            motor2.setMaxFrequency(newFrequency);
            Serial2.print("New max frequency set: ");
            Serial2.println(newFrequency);
        }
    }
    
    switch (currentState) {
        case STATE_HOMING:
            led.setColor(0, 64, 0, 0);
            home();
            break;
        case STATE_READY:
            led.setColor(0, 0, 0, 64);
            // 使用ButtonHandler类处理按键消抖
            if (startButton.isPressed()) {
                currentState = STATE_RUNNING;
                steps = 0;
            }
            break;
        case STATE_RUNNING:
            led.setColor(0, 0, 64, 0);
            run();
            break;
        case STATE_FAULT:
            fault();
            break;
    }
    delay(10);
}

void Error_Handler(void) {
    // 错误处理函数
    while(1) {
        Serial2.println("Error_Handler!");
        led.blinkEffect(0, 64, 0, 0, 500);
        // 添加看门狗喂狗操作
        watchdog.feed();
        delay(100);
    }
}