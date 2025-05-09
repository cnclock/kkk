#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class LEDHandler {
public:
    LEDHandler(uint8_t pin, uint16_t numPixels);
    void setColor(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
    void breatheEffect(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
    void blinkEffect(uint8_t i, uint8_t r, uint8_t g, uint8_t b, uint16_t interval);

private:
    Adafruit_NeoPixel strip;
};