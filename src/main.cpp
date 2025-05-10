#include "config.h"
#include "eeprom_manager.h"
#include "motor_controller.h"
//#include "ultrasonic.h"
#include "button_handler.h"
#include "led_handler.h"
#include "lcd_display.h"
#include "watchdog.h"
#include "pid_controller.h"

// 取消Error_Handler宏定义，避免与函数冲突
#undef Error_Handler

//#include "stm32f1xx_hal.h"  // 添加HAL库头文件  //已经在watchdog.h中包含
#include <Ultrasonic.h>

// 声明Error_Handler函数
void Error_Handler(void);

TIM_HandleTypeDef htim3;  // 使用TIM3
TIM_HandleTypeDef htim4;  // 使用TIM4


// 实例化对象
MotorController motor1(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_EN_PIN, LIMIT_SW1_NEG, &htim3, TIM_CHANNEL_1);
MotorController motor2(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_EN_PIN, LIMIT_SW2_NEG, &htim4, TIM_CHANNEL_1);
Ultrasonic ultrasonic1(US1_TRIG, US1_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(US2_TRIG, US2_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic3(US3_TRIG, US3_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic4(US4_TRIG, US4_ECHO);    // An ultrasonic sensor HC-04
ButtonHandler startButton(BUTTON_START);
LEDHandler led(LED_DATA_PIN, 3);
LCDDisplay lcd(lcdPins);
Watchdog watchdog(2000); // 实例化看门狗
//设置PID初始参数，包括比例系数kp，积分系数ki，微分系数kd，积分限制integralLimit，采样时间sampleTime
PIDController pid1(10.0, 0.0, 0.0, 100.0, 0.1); 
PIDController pid2(10.0, 0.0, 0.0, 100.0, 0.1); 
EEPROMManager eeprom;

SystemState currentState = STATE_HOMING;
uint16_t steps = 0;
void Error_Handler(void); // 函数声明
void home();
void run();
void fault();


void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** 初始化RCC振荡器 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /** 初始化RCC时钟 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}



// 定时器初始化函数（基于频率控制）
void TIM3_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    //TIM_HandleTypeDef htim3;  // 使用TIM3
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;      // 1MHz时钟
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;       // 1kHz频率
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  // 50%占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void TIM4_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72 - 1;      // 1MHz时钟
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 1000 - 1;       // 1kHz频率
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  // 50%占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void GPIO_Init() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TIM3 CH1 (PA6)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM4 CH1 (PB6)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void home() {
    static unsigned long lastFeedTime = millis();
    if (motor1.isHomed()) {
        Serial2.println("电机1: OK.");
    }
    else
    {
        motor1.startHome();
    }
    if (motor2.isHomed()) {
        Serial2.println("电机2: OK.");
    }
    else
    {
        motor2.startHome();
    }
    if (millis() - lastFeedTime > 10) {
        watchdog.feed();
        lastFeedTime = millis();
    }
    if (motor1.isHomed() && motor2.isHomed()) {
        currentState = STATE_READY;
        Serial2.println("就绪，请按开始键...");
    }
}

void run() {
    float distance1 = ultrasonic1.read();
    float distance2 = ultrasonic2.read();
    float distance3 = ultrasonic3.read();
    float distance4 = ultrasonic4.read();

    if (distance1 <= 0 || distance2 <= 0 && distance3 <= 0 || distance4 <= 0) {
        Serial2.println("Ultrasonic sensor error!");
        currentState = STATE_FAULT;
        return;
    }

    Serial2.println("运行中...");
    Serial2.print("distance1: ");
    Serial2.println(distance1);
    Serial2.print("distance2: ");
    Serial2.println(distance2);
    Serial2.print("distance3: ");
    Serial2.println(distance3);
    Serial2.print("distance4: ");
    Serial2.println(distance4);

    motor1.run(distance1, distance2, pid1);
    motor2.run(distance3, distance4, pid2);

    // 添加看门狗喂狗操作
    watchdog.feed();
}

void fault() {
    motor1.disable();
    motor2.disable();
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;

    if (startButton.isPressed()) {
        unsigned long currentTime = millis();
        if (currentTime - lastDebounceTime > debounceDelay) {
            currentState = STATE_HOMING;
        }
    }

    led.blinkEffect(0, 64, 0, 0, 500);
    watchdog.feed();
}

void setup() {
    currentState = STATE_HOMING;
    HAL_Init();  // 添加HAL库初始化
    SystemClock_Config();  // 系统时钟配置
    Serial.begin(115200);
    Serial2.begin(115200);
    led.setColor(0, 64, 0, 0);
    Serial2.println(" ");
    Serial2.println("系统启动...");
    Serial2.println("输出调试信息");
    Serial2.print("System Clock Frequency: ");
    Serial2.print(HAL_RCC_GetHCLKFreq());
    Serial2.println(" Hz");
    
    // 初始化看门狗
    Serial2.print("初始化看门狗... ");
    watchdog.start(); 
    Serial2.println("OK.");
    
    // 初始化电机控制器
    Serial2.print("初始化电机控制器... ");
    TIM3_Init(); // 初始化PWM输出
    TIM4_Init(); // 初始化PWM输出
    GPIO_Init(); // 初始化GPIO
    Serial2.println("OK.");
    
    // 初始化NVIC中断优先级
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    
    Serial2.println("回零...");
    home();
}

void loop() {
    watchdog.feed();
    
    if (Serial2.available() > 0) {
        int newFrequency = Serial2.parseInt();
        if (newFrequency > 0 && newFrequency <= 10000) { // 确保频率在有效范围内
            motor1.setMaxFrequency(newFrequency);
            motor2.setMaxFrequency(newFrequency);
            Serial2.print("New max frequency set: ");
            Serial2.println(newFrequency);
        } else {
            Serial2.println("Invalid frequency input!");
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
        // 检查是否按下复位按钮
        if (startButton.isPressed()) {
            Serial2.println("Exiting Error_Handler...");
            NVIC_SystemReset(); // 复位系统
        }
    }
}