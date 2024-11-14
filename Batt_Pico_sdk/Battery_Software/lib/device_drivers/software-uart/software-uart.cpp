#include "software-uart.h"
#include "uart_rx.pio.h"
#include <stdio.h>

SoftwareUART::SoftwareUART(uint pin) {
    printf("Initializing SoftwareUART on pin %d\n", pin);
    
    // Initialize sm to invalid value
    sm = -1;
    
    // Validate PIO and SM allocation
    pio = pio1;
    if (!pio) {
        printf("ERROR: Failed to get PIO instance\n");
        return;
    }

    sm = (int)pio_claim_unused_sm(pio, true);
    if (sm < 0) {
        printf("ERROR: Could not claim a SM\n");
        return;
    }
    printf("Using PIO1 with SM %d\n", sm);

    // Load program
    if (pio_can_add_program(pio, &uart_rx_program)) {
        offset = pio_add_program(pio, &uart_rx_program);
        printf("PIO program loaded at offset %d\n", offset);
    } else {
        printf("ERROR: Not enough PIO program space\n");
        sm = -1;
        return;
    }

    // Initialize pin
    pio_gpio_init(pio, pin);
    gpio_pull_up(pin);
    printf("Pin initialized\n");
    
    // Configure state machine
    pio_sm_config c = uart_rx_program_get_default_config(offset);
    
    // Set input pin
    sm_config_set_in_pins(&c, pin);
    
    // Important: Configure shifting
    sm_config_set_in_shift(&c, true, false, 8);  // right shift, no autopush
    sm_config_set_out_shift(&c, true, false, 8); // right shift, no autopull
    
    // Calculate divider
    float div = (float)clock_get_hz(clk_sys) / (SERIAL_BAUD * 8);
    printf("Setting clock divider to: %f\n", div);
    sm_config_set_clkdiv(&c, div);
    
    // Initialize SM with config
    pio_sm_init(pio, sm, offset, &c);
    
    // Clear FIFOs before enabling
    pio_sm_clear_fifos(pio, sm);
    
    printf("Starting state machine\n");
    pio_sm_set_enabled(pio, sm, true);
    
    // Verify final setup
    printf("Setup complete. FIFO empty: %d, RX FIFO level: %d\n",
           pio_sm_is_rx_fifo_empty(pio, sm),
           pio_sm_get_rx_fifo_level(pio, sm));
}

uint8_t SoftwareUART::receiveBytes() {
    if (!pio || sm < 0) {
        printf("ERROR: UART not properly initialized\n");
        return 0;
    }

    printf("Waiting for byte...\n");
    
    const uint32_t TIMEOUT_MS = 1000;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - start_time > TIMEOUT_MS) {
            printf("Timeout waiting for byte\n");
            printf("PIO status - RX FIFO empty: %d\n", pio_sm_is_rx_fifo_empty(pio, sm));
            printf("PIO status - RX FIFO level: %d\n", pio_sm_get_rx_fifo_level(pio, sm));
            printf("PIO program counter: 0x%x\n", pio_sm_get_pc(pio, sm));
            return 0;
        }

        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t raw_data = pio_sm_get(pio, sm);
            printf("Raw data: 0x%08x\n", raw_data);
            
            // The data is shifted into the MSB, so we need to shift it back
            uint8_t received_byte = (raw_data >> 24) & 0xFF;
            
            printf("Decoded byte: 0x%02x ('%c')\n", received_byte, 
                   (received_byte >= 32 && received_byte <= 126) ? received_byte : '.');
            
            return received_byte;
        }
        
        sleep_ms(1);
    }
}

SoftwareUART::~SoftwareUART() {
    if (pio && sm >= 0) {
        pio_sm_set_enabled(pio, sm, false);
        pio_remove_program(pio, &uart_rx_program, offset);
        pio_sm_unclaim(pio, sm);
    }
}