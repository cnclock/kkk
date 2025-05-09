#include "eeprom_manager.h"
#include <CRC32.h>

EEPROMManager::EEPROMManager() {
    // 初始化操作
}

bool EEPROMManager::writeData(const uint8_t* data, size_t size) {
    uint32_t crc = calculateCRC32(data, size);
    // 写入数据和CRC
    // 这里需要实现磨损均衡
    return true;
}

bool EEPROMManager::readData(uint8_t* data, size_t size) {
    // 读取数据和CRC
    uint32_t readCRC;
    uint32_t calculatedCRC = calculateCRC32(data, size);
    if (readCRC == calculatedCRC) {
        return true;
    }
    return false;
}

uint32_t EEPROMManager::calculateCRC32(const uint8_t* data, size_t size) {
    CRC32 crc;
    crc.update(data, size);
    return crc.finalize();
}