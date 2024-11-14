#include "software-uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

SoftwareUART::SoftwareUART(uint pin) : 
    rx_pin(pin),
    bit_time_us(104),      // 1000000/9600 ≈ 104.167 microseconds per bit
    half_bit_time_us(52),
    receiving(false) {
    
    gpio_init(rx_pin);
    gpio_set_dir(rx_pin, GPIO_IN);
    gpio_pull_up(rx_pin);  // Enable pull-up resistor
}

uint8_t SoftwareUART::receiveBytes() {
    uint8_t data = 0;
    
    // Wait for start bit (falling edge)
    while (gpio_get(rx_pin) == 1) {
        tight_loop_contents();
    }
    
    // Synchronize to middle of start bit
    busy_wait_us(half_bit_time_us);
    
    // Verify we're still in start bit
    if (gpio_get(rx_pin) != 0) {
        return 0xFF;  // False start
    }
    
    receiving = true;
    
    // Wait for start bit to finish
    busy_wait_us(half_bit_time_us);
    
    // Read 8 data bits
    for (int i = 0; i < 8; i++) {
        data >>= 1;  // Shift right to make room for next bit
        
        // Sample in middle of bit
        if (gpio_get(rx_pin)) {
            data |= 0x80;  // Set MSB if pin is high
        }
        
        busy_wait_us(bit_time_us);
    }
    
    // Wait for stop bit
    if (gpio_get(rx_pin) == 0) {
        return 0xFF;  // Framing error
    }
    
    receiving = false;
    return data;
}

bool SoftwareUART::isReceiving() const {
    return receiving;
}
