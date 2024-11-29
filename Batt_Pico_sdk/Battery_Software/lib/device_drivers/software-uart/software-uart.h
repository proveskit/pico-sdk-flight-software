// software_uart.h
#ifndef SOFTWARE_UART_H
#define SOFTWARE_UART_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"

class SoftwareUART {
public:
    /**
     * @brief Construct a new Software UART object
     * 
     * @param pin GPIO pin number to use for RX
     */
    explicit SoftwareUART(uint pin);
    
    /**
     * @brief Receive a single byte via software UART
     * 
     * @return uint8_t Received byte, or 0xFF if error occurred
     */
    uint8_t receiveBytes();
    
    /**
     * @brief Check if currently receiving data
     * 
     * @return true if in the middle of receiving a byte
     */
    bool isReceiving() const;

private:
    const uint rx_pin;
    const uint32_t bit_time_us;     // Time for one bit at 9600 baud
    const uint32_t half_bit_time_us; // Half of the bit time for centering samples
    volatile bool receiving;         // Flag indicating active reception
};

#endif // SOFTWARE_UART_H