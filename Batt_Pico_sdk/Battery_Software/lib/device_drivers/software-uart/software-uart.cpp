#include "software-uart.h"
#include "uart_rx.pio.h"
#include <stdio.h>

#define PIO_RX_PIN 26
#define SERIAL_BAUD 9600
SoftwareUART::SoftwareUART(uint pin) {
    pio = pio1;
    sm = 0;
    offset = pio_add_program(pio, &uart_rx_program);
    uart_rx_program_init(pio, sm, offset, PIO_RX_PIN, SERIAL_BAUD);
}

uint8_t SoftwareUART::receiveBytes() {
    printf("Waiting for byte...\n");
    while (true) {
        char c = uart_rx_program_getc(pio, sm);
        printf("Received: %c\n", c);
    }
}