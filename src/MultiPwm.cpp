#include "MultiPwm.h"

const MultiPwm::TimerMap MultiPwm::timerMap[4] = {
    {PA0, TIM2, 1}, // TIM2_CH1
    {PA6, TIM3, 1}, // TIM3_CH1
    {PB6, TIM4, 1}, // TIM4_CH1
    {PA8, TIM1, 1}  // TIM1_CH1
};

MultiPwm::MultiPwm(const uint8_t* pins, size_t num) : pwmPins(pins), pwmNum(num) {
    timers = new HardwareTimer*[pwmNum];
    for (size_t i = 0; i < pwmNum; ++i) {
        timers[i] = nullptr;
    }
}

MultiPwm::~MultiPwm() {
    for (size_t i = 0; i < pwmNum; ++i) {
        if (timers[i]) delete timers[i];
    }
    delete[] timers;
}

HardwareTimer* MultiPwm::getTimerByPin(uint8_t pin) {
    for (size_t i = 0; i < 4; i++) {
        if (timerMap[i].pin == pin) {
            return new HardwareTimer(timerMap[i].timer);
        }
    }
    return nullptr;
}

uint8_t MultiPwm::getTimerChannelByPin(uint8_t pin) {
    for (size_t i = 0; i < 4; i++) {
        if (timerMap[i].pin == pin) {
            return timerMap[i].channel;
        }
    }
    return 1;
}

void MultiPwm::begin(uint32_t freq, float duty) {
    for (size_t i = 0; i < pwmNum; i++) {
        timers[i] = getTimerByPin(pwmPins[i]);
        if (timers[i]) {
            pinMode(pwmPins[i], OUTPUT); //Arduino库的OUTPUT默认是推挽
            uint8_t channel = getTimerChannelByPin(pwmPins[i]);
            timers[i]->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pwmPins[i]);
            
            // 设置合适的频率并处理潜在的溢出
            setFrequency(i, freq);
            
            timers[i]->setCaptureCompare(channel, duty, PERCENT_COMPARE_FORMAT);
            timers[i]->resume();
        }
    }
}

void MultiPwm::setFrequency(uint8_t ch, uint32_t freq) {
    if (ch >= pwmNum || freq < 1) return;
    
    HardwareTimer *timer = timers[ch];
    if (!timer) return;
    
    // 先停止定时器
    timer->pause();
    
    // STM32F1定时器主频
    uint32_t timer_clk = 72000000;
    
    // 对于10kHz以内的频率，使用简化的固定分频方案
    uint32_t psc = 0;
    uint32_t arr = 0;
    
    // 根据不同的频率范围选择合适的预分频值
    if (freq <= 10) {
        // 对于非常低的频率 (≤10Hz)，使用大分频值
        psc = 7199;  // 7200分频
    } else if (freq <= 100) {
        // 对于低频 (≤100Hz)
        psc = 719;   // 720分频
    } else if (freq <= 1000) {
        // 对于中频 (≤1000Hz)
        psc = 71;    // 72分频
    } else {
        // 对于较高频率 (>1000Hz & ≤10kHz)
        psc = 7;     // 8分频
    }
    
    // 计算ARR值
    arr = (timer_clk / ((psc + 1) * freq)) - 1;
    
    // 确保ARR在有效范围内 (36-65535)
    if (arr > 65535) arr = 65535;
    if (arr < 36) arr = 36;
    
    // 计算实际输出频率
    uint32_t actual_freq = timer_clk / ((psc + 1) * (arr + 1));
    
    // 使用HAL库设置定时器寄存器
    TIM_TypeDef *tim = (TIM_TypeDef *)(timer->getHandle()->Instance);
    tim->PSC = psc;
    tim->ARR = arr;
    tim->CNT = 0;
    tim->EGR = TIM_EGR_UG;
    
    // 重启定时器
    timer->resume();
    
    // 计算误差百分比用于调试输出
    float error_percent = 100.0f * (float)(actual_freq - freq) / (float)freq;
    
}

void MultiPwm::setDuty(uint8_t ch, float duty) {
    if (ch >= pwmNum || duty < 0 || duty > 100) return;
    uint8_t channel = getTimerChannelByPin(pwmPins[ch]);
    timers[ch]->setCaptureCompare(channel, duty, PERCENT_COMPARE_FORMAT);
}