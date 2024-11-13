// VL6180.h

#ifndef VL6180_H
#define VL6180_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class VL6180 {
public:
    VL6180(i2c_inst_t* i2c_instance, uint8_t address = 0x29);
    bool begin();
    uint8_t getDistance();
    
private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    
    bool writeRegister(uint16_t reg, uint8_t value);
    uint8_t readRegister(uint16_t reg);
    void writeRegister16(uint16_t reg, uint16_t value);
    uint16_t readRegister16(uint16_t reg);
};

#endif // VL6180_H