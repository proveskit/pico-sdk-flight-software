#include "software-uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstdio>

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
    
    printf("Waiting for While\n");
    // Wait for start bit (falling edge)
    int i = 0;
    while (gpio_get(rx_pin) == 1) {
        i++;
        if (i == 1000) {
            printf("Timeout waiting for start bit\n");
            return 0xFF;  // Timeout
        }
        sleep_ms(1);
    }
    printf("Start bit detected\n");
    
    // Synchronize to middle of start bit
    busy_wait_us(half_bit_time_us);
    
    // Verify we're still in start bit
    if (gpio_get(rx_pin) != 0) {
        printf("False start detected\n");
        return 0xFF;  // False start
    }
    
    receiving = true;
    printf("Receiving data\n");
    
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
        printf("Bit %d received: %d\n", i, (data & 0x80) ? 1 : 0);
    }
    
    // Wait for stop bit
    if (gpio_get(rx_pin) == 0) {
        printf("Framing error detected\n");
        return 0xFF;  // Framing error
    }
    
    receiving = false;
    printf("Data received: 0x%02X\n", data);
    return data;
}

bool SoftwareUART::isReceiving() const {
    return receiving;
}
