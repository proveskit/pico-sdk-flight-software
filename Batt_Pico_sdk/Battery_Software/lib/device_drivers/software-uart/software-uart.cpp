#include "software-uart.h"
#include "uart_rx.pio.h"
#include <stdio.h>

SoftwareUART::SoftwareUART(uint pin) {
    printf("Initializing SoftwareUART on pin %d\n", pin);
    
    sm = -1;
    pio = pio1;
    
    if (!pio) {
        printf("ERROR: Failed to get PIO instance\n");
        return;
    }

    // Debug output for pin state
    printf("Initial pin state: %d\n", gpio_get(pin));

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

    // Initialize pin with debug output
    printf("Configuring pin %d for PIO\n", pin);
    pio_gpio_init(pio, pin);
    gpio_pull_up(pin);
    printf("Pin state after config: %d\n", gpio_get(pin));
    
    // Configure state machine with debug output
    pio_sm_config c = uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    
    // Calculate and verify baud rate
    float div = (float)clock_get_hz(clk_sys) / (8 * SERIAL_BAUD);
    printf("Clock divider: %f (system clock: %lu Hz)\n", div, clock_get_hz(clk_sys));
    sm_config_set_clkdiv(&c, div);
    
    sm_config_set_in_shift(&c, true, true, 8);
    
    pio_sm_init(pio, sm, offset, &c);
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
    
    // Add timeout (e.g., 1 second = 1000ms)
    const uint32_t TIMEOUT_MS = 1000;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        // Check for timeout
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - start_time > TIMEOUT_MS) {
            printf("Timeout waiting for byte\n");
            // Debug PIO state
            printf("PIO status - RX FIFO empty: %d\n", pio_sm_is_rx_fifo_empty(pio, sm));
            printf("PIO status - RX FIFO level: %d\n", pio_sm_get_rx_fifo_level(pio, sm));
            printf("PIO program counter: 0x%x\n", pio_sm_get_pc(pio, sm));
            return 0;
        }

        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t raw_data = pio_sm_get(pio, sm);
            char c = raw_data & 0xFF;
            printf("Received raw data: 0x%08x, char: %c\n", raw_data, c);
            return c;
        }
        
        sleep_ms(1);
    }
}

SoftwareUART::~SoftwareUART() {
    if (pio && sm >= 0) {  // Changed comparison
        pio_sm_set_enabled(pio, sm, false);
        pio_remove_program(pio, &uart_rx_program, offset);
        pio_sm_unclaim(pio, sm);
    }
}