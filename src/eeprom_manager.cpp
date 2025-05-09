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

// eeprom_manager.cpp
bool EEPROMManager::readData(uint8_t* data, size_t size) {
    uint32_t storedCRC;
    // 从EEPROM读取存储的CRC
    // 示例: EEPROM.get(address, storedCRC);
    
    uint32_t calculatedCRC = calculateCRC32(data, size);
    return (storedCRC == calculatedCRC);
}

uint32_t EEPROMManager::calculateCRC32(const uint8_t* data, size_t size) {
    CRC32 crc;
    crc.update(data, size);
    return crc.finalize();
}