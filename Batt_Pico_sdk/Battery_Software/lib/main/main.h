#ifndef _MAIN_H
#define _MAIN_H

//Modified with Claude 3.5 Sonnet

// Standard library includes
#include <iostream>
#include <stdio.h>
#include <cstring>

// Custom library includes
#include <neopixel/neopixel.h>
#include <tools/tools.h>
#include <pysquared/pysquared.h>
#include <functions/functions.h>

// Hardware includes
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "hardware/flash.h"
#include "pico/stdlib.h"

// Time constants (in milliseconds)
constexpr uint WATCHDOG_TIMEOUT_MS = 5000;    // 5 seconds
constexpr uint LOITER_TIME_SECONDS = 270;     // 4.5 minutes
constexpr uint SLEEP_INTERVAL_MS = 1000;      // 1 second
constexpr uint BLINK_INTERVAL_MS = 500;       // 0.5 seconds

// Register addresses
constexpr uint8_t STATUS_REG = 1;
constexpr uint8_t BOOT_REG = 2;

// Status register bit positions
constexpr uint8_t BROWNOUT_BIT = 0;
constexpr uint8_t WATCHDOG_BIT = 1;
constexpr uint8_t BURNED_BIT = 2;
constexpr uint8_t HEATER_LATCH_BIT = 3;
constexpr uint8_t VBUS_RESET_BIT = 4;
constexpr uint8_t PRIOR_BURN_ATTEMPT = 5;

// Instead of uint32_t, define separate RGB values
struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// LED Colors as RGB structs
constexpr RGB LED_GREEN  = {0x00, 0xFF, 0x00};
constexpr RGB LED_RED    = {0xFF, 0x00, 0x00};
constexpr RGB LED_YELLOW = {0xFF, 0xFF, 0x00};
constexpr RGB LED_PURPLE = {0xFF, 0x00, 0xFF};
constexpr RGB LED_OFF    = {0x00, 0x00, 0x00};

using namespace std;

// Struct for burn status tracking
struct BurnStatus {
    bool has_burned_before;
    bool previous_brownout;
};

// Function declarations
void main_program(neopixel neo);

// Initialization functions
bool initializeWatchdog(tools& t);
void handleFlashInitialization(pysquared& satellite, tools& t, bool watchdog_reset, uint8_t* data);
BurnStatus checkBurnStatus(pysquared& satellite, tools& t, uint8_t* data);
void updateBootCount(pysquared& satellite, tools& t, uint8_t* data);

// Operation functions
bool executeBurnSequence(pysquared& satellite, satellite_functions& functions, 
                        tools& t, neopixel& neo, bool has_burned_before);
void runMainLoop(pysquared& satellite, satellite_functions& functions, 
                tools& t, neopixel& neo);
void test_can(pysquared satellite, neopixel neo, satellite_functions functions);

// Power mode operation functions
void critical_power_operations(tools t, satellite_functions functions);
void low_power_operations(tools t, neopixel neo, satellite_functions functions);
void normal_power_operations(tools t, neopixel neo, satellite_functions functions);
void maximum_power_operations(tools t, neopixel neo, satellite_functions functions);

#endif