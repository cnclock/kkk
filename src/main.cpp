#include "config.h"
#include "eeprom_manager.h"
#include "motor_controller.h"
#include "button_handler.h"
#include "led_handler.h"
///#include "lcd_display.h"
#include "watchdog.h"
#include "pid_controller.h"
#include "MultiPwm.h"
#include <Ultrasonic.h>

// 取消Error_Handler宏定义，避免与函数冲突
#undef Error_Handler

// 实例化对象
MultiPwm pwm(PWM_PINS, 4);
MotorController motor1(PWM_PINS[2], DIR_PINS[2], EN_PINS[2], LIMIT_SW1_NEG, 2); // 通道2
MotorController motor2(PWM_PINS[3], DIR_PINS[3], EN_PINS[3], LIMIT_SW2_NEG, 3); // 通道3
MotorController motor3(PWM_PINS[1], DIR_PINS[1], EN_PINS[1], LIMIT_SW1_NEG, 1); // 通道1
MotorController motor4(PWM_PINS[0], DIR_PINS[0], EN_PINS[0], LIMIT_SW2_NEG, 0); // 通道0
Ultrasonic ultrasonic1(US1_TRIG, US1_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(US2_TRIG, US2_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic3(US3_TRIG, US3_ECHO);    // An ultrasonic sensor HC-04
Ultrasonic ultrasonic4(US4_TRIG, US4_ECHO);    // An ultrasonic sensor HC-04
ButtonHandler startButton(BUTTON_START);
LEDHandler led(LED_DATA_PIN, 3);
Watchdog watchdog(2000); // 实例化看门狗
//设置PID初始参数，包括比例系数kp，积分系数ki，微分系数kd，积分限制integralLimit，采样时间sampleTime
PIDController pid1(10.0, 0.05, 0.03, 100.0, 0.1);
PIDController pid2(10.0, 0.05, 0.03, 100.0, 0.1);
float Kc = 0.5;                 // 交叉耦合系数
float e_couple;
bool isRunning = false; // 是否运行标志

EEPROMManager eeprom;

SystemState currentState = STATE_HOMING;
void Error_Handler(void); // 函数声明
void home();
void run();
void fault();
void move(bool isMove);
void brush(bool isBrushOn);
void feedWatchdog();
void printDebug(const String &message); // 用于打印调试信息
bool checkTimeElapsed(unsigned long &lastTime, unsigned long interval); // 检查时间是否已经过去


void GPIO_Init() {

   pinMode(BRUSH_PIN, OUTPUT_OPEN_DRAIN); // 设置继电器引脚为开漏输出模式
   digitalWrite(BRUSH_PIN, HIGH); // 设置继电器高电平

}

void home() {
    static bool motor1OkPrinted = false;
    static bool motor2OkPrinted = false;

    feedWatchdog();

    // 处理电机回零
    auto processMotorHoming = [](MotorController &motor, bool &motorOkPrinted, const char* motorName) {
        if (motor.isHomed()) {
            if (!motorOkPrinted) {
                motorOkPrinted = true;
                Serial2.println(String(motorName) + ":  OK.");
            }
        } else {
            motor.startHome();
            motorOkPrinted = false;
        }
    };

    // 电机1和电机2回零
    processMotorHoming(motor1, motor1OkPrinted, "Motor 1");
    processMotorHoming(motor2, motor2OkPrinted, "Motor 2");

    if (motor1OkPrinted && motor2OkPrinted) {
        Serial2.println("----- HOMING COMPLETE -----");
        currentState = STATE_READY;
        Serial2.println("HOMING COMPLETE. Ready! Press Start Button");
    }
}

void run() {
    bool isBrushOn = false; // 是否刷子开启标志
    bool isMove = true; // 是否移动标志
    static bool isStable = false; // 是否稳态标志
    static uint8_t stableCount = 0; // 稳态计数器
    
    feedWatchdog();
    static unsigned long lastFeedButtonTime = millis();

    // 读取所有超声波传感器数据
    float distance1 = ultrasonic1.read();
    float distance2 = ultrasonic2.read();
    float distance3 = ultrasonic3.read();
    float distance4 = ultrasonic4.read();
    
    // 检查安全距离
    bool isSafetyViolated = distance1 > SAFETY_DISTANCE || 
                          distance2 > SAFETY_DISTANCE || 
                          distance3 > SAFETY_DISTANCE || 
                          distance4 > SAFETY_DISTANCE;
    
    if (isSafetyViolated) {
        printDebug("UltrasonicSensor");
        printDebug("Error! > SAFETY_DISTANCE");
        printDebug("distance1: " + String(distance1) +
                  ", distance2: " + String(distance2) +
                  ", distance3: " + String(distance3) +
                  ", distance4: " + String(distance4));
        currentState = STATE_FAULT;
        return;
    }

    // 检测是否在启动距离范围内
    bool isWithinStartDistance = (distance1 < START_DISTANCE || distance2 < START_DISTANCE) && 
                               (distance3 < START_DISTANCE || distance4 < START_DISTANCE);
                               
    isRunning = isWithinStartDistance;
    isMove = !isWithinStartDistance;

    float distanceA = min(distance1, distance2);
    float distanceB = min(distance3, distance4);
    e_couple = Kc * (distanceA - distanceB);

    if (isRunning) {
        motor1.run(distanceA + e_couple, pid1);
        motor2.run(distanceB - e_couple, pid2);
        
        bool bothPidsStable = pid1.isStable(STABLE_DISTANCE, STABLE_TIME) && 
                             pid2.isStable(STABLE_DISTANCE, STABLE_TIME);
                             
        if (bothPidsStable) {
            isMove = true;
            isBrushOn = true;
            isStable = true;
        } else {
            isBrushOn = false;
            isMove = false;
        }
        
        // 检查是否按下复位按钮
        if (startButton.isPressed()) {
            if (millis() - lastFeedButtonTime > 1000) {
                printDebug("Resetting...");
                NVIC_SystemReset(); // 复位系统                
                lastFeedButtonTime = millis();
            }
        }
    } else {
        motor1.stop();
        motor2.stop();
        isMove = true;
    }
    
    move(isMove); // 调用移动函数
    brush(isBrushOn); // 调用刷子控制函数
    
    // 检测稳态错误
    float distanceDifference = abs(distance1 + distance2 - distance3 - distance4);
    if (isStable && (distanceDifference > 50)) {
        stableCount++;
        if (stableCount >= STABLE_COUNT) {
            printDebug(String(distanceDifference) + " Error! > 50mm");
            NVIC_SystemReset(); // 复位系统
        }
    }
}

void brush(bool isBrushOn) {
    static unsigned long lastFeedDelayTime = millis();
    feedWatchdog();
    
    if (isBrushOn) {
        digitalWrite(BRUSH_PIN, LOW); 
    } else {
        if (checkTimeElapsed(lastFeedDelayTime, 2000)) {
            digitalWrite(BRUSH_PIN, HIGH);
            lastFeedDelayTime = millis();
        }
    }
}

void move(bool isMove) {
    static unsigned long lastFeedDelayTime = millis();
    feedWatchdog();
    
    if (isMove) {
        motor3.run(TARGET_DISTANCE - MOVE_SPEED, pid1); // 行走电机3运行
        motor4.run(TARGET_DISTANCE + MOVE_SPEED, pid2); // 行走电机4运行
    } else {
        if (checkTimeElapsed(lastFeedDelayTime, 4000)) {
            motor3.stop();
            motor4.stop();
            lastFeedDelayTime = millis();
        }        
    }
}

void fault() {
    motor1.disable();
    motor2.disable();
    motor3.disable();
    motor4.disable();

    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50;

    if (startButton.isPressed()) {
        unsigned long currentTime = millis();
        if (currentTime - lastDebounceTime > debounceDelay) {
            currentState = STATE_HOMING;
        }
    }

    led.blinkEffect(0, 64, 0, 0, 500);
    feedWatchdog();
}

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600);
    led.setColor(0, 64, 0, 0);

    printDebug(" ");
    printDebug("系统启动...");
    printDebug("输出调试信息");
    Serial2.print("System Clock Frequency: ");
    Serial2.print(F_CPU);
    printDebug(" Hz");

    // 初始化看门狗
    Serial2.print("Init watchdog..");
    watchdog.start();
    Serial2.println(" OK.");

    // 初始化电机控制器
    Serial2.print("Init Motor... ");
    pwm.begin(100,50);  // 1Hz, 50% duty cycle
    GPIO_Init(); // 初始化GPIO
    Serial2.println(" OK.");
    watchdog.feed();

    currentState = STATE_HOMING;
    Serial2.println("Homing...");
}

void loop() {
    feedWatchdog();

    if (Serial2.available() > 0) {
        int newFrequency = Serial2.parseInt();
        if (newFrequency > 0 && newFrequency <= MAX_FREQUENCY) { // 确保频率在有效范围内
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
            if (startButton.isPressed()) {
                currentState = STATE_RUNNING;
                isRunning = false; // 设置运行标志为false
                printDebug("System Start...");

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
void feedWatchdog() {
    static unsigned long lastFeedTime = millis();
    if (millis() - lastFeedTime > 100) {
        watchdog.feed();
        lastFeedTime = millis();
    }
}

void printDebug(const String &message) {
    Serial2.println(message);
}

bool checkTimeElapsed(unsigned long &lastTime, unsigned long interval) {
    return (millis() - lastTime) > interval;
}

void Error_Handler(void) {
    // 错误处理函数
    while(1) {
        Serial2.print("SET_CURSOR 0 0 ");
        Serial2.println("Error_Handler!");
        led.blinkEffect(0, 64, 0, 0, 500);
        // 添加看门狗喂狗操作
        feedWatchdog();
        delay(100);
        // 检查是否按下复位按钮
        if (startButton.isPressed()) {
            Serial2.print("SET_CURSOR 0 0 ");
            Serial2.println("Resetting...");
            //Serial2.println("Exiting Error_Handler...");
            NVIC_SystemReset(); // 复位系统
        }
    }
}