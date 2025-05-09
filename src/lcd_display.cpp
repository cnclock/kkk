#include "lcd_display.h"

LCDDisplay::LCDDisplay(const uint8_t* pins)
    : lcd(pins[0], pins[1], pins[2], pins[3], pins[4], pins[5]) {
    lcd.begin(16, 2);
}

void LCDDisplay::displayMessage(const char* message, uint8_t row, uint8_t col) {
    lcd.setCursor(col, row);
    lcd.print(message);
}