// VL6180.cpp

#include "VL6180.h"

// VL6180 registers
#define VL6180X_SYSTEM_INTERRUPT_CONFIG 0x014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR 0x015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET 0x016
#define VL6180X_SYSRANGE_START 0x018
#define VL6180X_RESULT_INTERRUPT_STATUS 0x04f
#define VL6180X_RESULT_RANGE_VAL 0x062

VL6180::VL6180(i2c_inst_t* i2c_instance, uint8_t address){
    _i2c = i2c_instance;
    _address = address;
    }

bool VL6180::begin() {
    // Check if the sensor is responding
    if (readRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET) != 1) {
        return false;
    }

    // Mandatory register writes for initialization
    writeRegister16(0x0207, 0x01);
    writeRegister16(0x0208, 0x01);
    writeRegister16(0x0096, 0x00);
    writeRegister16(0x0097, 0xfd);
    writeRegister16(0x00e3, 0x00);
    writeRegister16(0x00e4, 0x04);
    writeRegister16(0x00e5, 0x02);
    writeRegister16(0x00e6, 0x01);
    writeRegister16(0x00e7, 0x03);
    writeRegister16(0x00f5, 0x02);
    writeRegister16(0x00d9, 0x05);
    writeRegister16(0x00db, 0xce);
    writeRegister16(0x00dc, 0x03);
    writeRegister16(0x00dd, 0xf8);
    writeRegister16(0x009f, 0x00);
    writeRegister16(0x00a3, 0x3c);
    writeRegister16(0x00b7, 0x00);
    writeRegister16(0x00bb, 0x3c);
    writeRegister16(0x00b2, 0x09);
    writeRegister16(0x00ca, 0x09);
    writeRegister16(0x0198, 0x01);
    writeRegister16(0x01b0, 0x17);
    writeRegister16(0x01ad, 0x00);
    writeRegister16(0x00ff, 0x05);
    writeRegister16(0x0100, 0x05);
    writeRegister16(0x0199, 0x05);
    writeRegister16(0x01a6, 0x1b);
    writeRegister16(0x01ac, 0x3e);
    writeRegister16(0x01a7, 0x1f);
    writeRegister16(0x0030, 0x00);

    return true;
}

uint8_t VL6180::getDistance() {
    // Start a single range measurement
    writeRegister(VL6180X_SYSRANGE_START, 0x01);

    // Wait for the measurement to complete
    while (!(readRegister(VL6180X_RESULT_INTERRUPT_STATUS) & 0x04)) {
        sleep_ms(1);
    }

    // Read the range result
    uint8_t range = readRegister(VL6180X_RESULT_RANGE_VAL);

    // Clear the interrupt
    writeRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

    return range;
}

bool VL6180::writeRegister(uint16_t reg, uint8_t value) {
    uint8_t buffer[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
    return i2c_write_blocking(_i2c, _address, buffer, 3, false) == 3;
}

uint8_t VL6180::readRegister(uint16_t reg) {
    uint8_t buffer[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    i2c_write_blocking(_i2c, _address, buffer, 2, true);
    uint8_t value;
    i2c_read_blocking(_i2c, _address, &value, 1, false);
    return value;
}

void VL6180::writeRegister16(uint16_t reg, uint16_t value) {
    writeRegister(reg, (uint8_t)(value >> 8));
    writeRegister(reg + 1, (uint8_t)(value & 0xFF));
}

uint16_t VL6180::readRegister16(uint16_t reg) {
    uint16_t value = readRegister(reg) << 8;
    value |= readRegister(reg + 1);
    return value;
}