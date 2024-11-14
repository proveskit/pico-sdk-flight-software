#ifndef SOFTWARE_UART_H
#define SOFTWARE_UART_H

#include "hardware/pio.h"
#include <cstdint>

class SoftwareUART {
private:
    uint rx_pin;

    // PIO related members
    PIO pio;
    uint sm;
    uint offset;

public:
    SoftwareUART(uint pin);
    uint8_t receiveBytes();

};

#endif // SOFTWARE_UART_H