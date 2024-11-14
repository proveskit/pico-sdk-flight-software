#ifndef SOFTWARE_UART_H
#define SOFTWARE_UART_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

class SoftwareUART {
private:
    PIO pio;
    int sm;  // Changed from uint to int
    uint offset;
    static const uint SERIAL_BAUD = 9600;

public:
    SoftwareUART(uint pin);
    ~SoftwareUART();
    uint8_t receiveBytes();
    bool isInitialized() { return pio != nullptr && sm >= 0; }  // Changed comparison
};

#endif