#pragma once
#include <Arduino.h>
//#include <Adafruit_LiquidCrystal.h>
#include <LiquidCrystal.h>

class LCDDisplay {
public:
    LCDDisplay(const uint8_t* pins);
    void displayMessage(const char* message, uint8_t row, uint8_t col);

private:
    LiquidCrystal lcd;
};