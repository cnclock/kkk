#pragma once
#include <Arduino.h>

class EEPROMManager {
public:
    EEPROMManager();
    bool writeData(const uint8_t* data, size_t size);
    bool readData(uint8_t* data, size_t size);

private:
    uint32_t calculateCRC32(const uint8_t* data, size_t size);
};