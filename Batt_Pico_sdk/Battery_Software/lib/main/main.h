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
#include "pico/stdio_usb.h"

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
void general_operations(tools t, satellite_functions functions);
void critical_power_operations(tools t, satellite_functions functions);
void low_power_operations(tools t, neopixel neo, satellite_functions functions);
void normal_power_operations(tools t, neopixel neo, satellite_functions functions);
void maximum_power_operations(tools t, neopixel neo, satellite_functions functions);

/*
TEST Keyboard Entry Stuff
*/

// Add these to main.h:
#define MAX_COMMAND_LENGTH 32
#define COMMAND_BUFFER_SIZE 256

class CommandSystem {
private:
    char input_buffer[COMMAND_BUFFER_SIZE];
    volatile int buffer_pos = 0;
    volatile bool line_received = false;
    pysquared& satellite;
    satellite_functions& functions;
    tools& t;
    neopixel& neo;

public:
    CommandSystem(pysquared& sat, satellite_functions& func, tools& tools, neopixel& neopix) 
        : satellite(sat), functions(func), t(tools), neo(neopix) {
        stdio_init_all();  // Initialize stdio for USB
        while (!stdio_usb_connected()) {
            sleep_ms(100);  // Wait for USB connection
        }
    }

    void process_input() {
        int c = getchar_timeout_us(0);  // Non-blocking getchar
        
        if (c != PICO_ERROR_TIMEOUT) {
            // Handle carriage return or newline
            if (c == '\r' || c == '\n') {
                if (buffer_pos > 0) {  // Only process if we have actual input
                    printf("\n");  // Move to new line
                    input_buffer[buffer_pos] = '\0';
                    line_received = true;
                    buffer_pos = 0;
                }
            }
            // Handle backspace
            else if (c == 0x7F || c == 0x08) {
                if (buffer_pos > 0) {
                    buffer_pos--;
                    printf("\b \b");  // Erase character from terminal
                }
            }
            // Handle printable characters
            else if (c >= 32 && c <= 126 && buffer_pos < COMMAND_BUFFER_SIZE - 1) {
                input_buffer[buffer_pos++] = c;
                putchar(c);  // Echo character
            }
        }

        if (line_received) {
            execute_command();
            line_received = false;
            printf("\n> ");
            fflush(stdout);
        }
    }

    void print_help() {
        printf("\r\nAvailable Commands:\r\n");
        printf("help            - Show this help message\r\n");
        printf("status          - Show satellite status\r\n");
        printf("battery         - Run battery manager\r\n");
        printf("burn           - Test burn sequence\r\n");
        printf("fc <on/off>     - Turn flight computer on/off\r\n");
        printf("5v <on/off>     - Toggle 5V power\r\n");
        printf("lidar           - Get LiDAR reading\r\n");
        printf("led <color>     - Set LED (red/green/yellow/purple/off)\r\n");
        printf("faces <on/off>  - Control all faces\r\n");
        printf("camera <on/off> - Control camera\r\n");
        printf("\r\n> ");
        fflush(stdout);
    }

private:
    // Helper function to generate a command hash at compile time
    constexpr static uint32_t hash_str(const char* str) {
        uint32_t hash = 5381;
        while (*str) {
            hash = ((hash << 5) + hash) + *str++;
        }
        return hash;
    }

    // Helper function to hash at runtime
    uint32_t hash_runtime(const char* str) {
        uint32_t hash = 5381;
        while (*str) {
            hash = ((hash << 5) + hash) + *str++;
        }
        return hash;
    }

    void execute_command() {
        char cmd[MAX_COMMAND_LENGTH];
        char arg[MAX_COMMAND_LENGTH];
        int parsed = sscanf(input_buffer, "%s %s", cmd, arg);

        if (parsed <= 0) {
            return; // Empty line
        }

        // Hash the command string for switch case
        switch (hash_runtime(cmd)) {
            case hash_str("help"):
                print_help();
                break;

            case hash_str("status"):
                printf("\r\nPower Mode: %d\r\n", satellite.power_mode());
                printf("Burned: %s\r\n", satellite.burned ? "Yes" : "No");
                printf("Armed: %s\r\n", satellite.is_armed() ? "Yes" : "No");
                break;

            case hash_str("battery"):
                printf("\r\nRunning battery manager...\r\n");
                functions.battery_manager();
                break;

            case hash_str("burn"):
                if (satellite.is_armed()) {
                    printf("\r\nInitiating burn sequence...\r\n");
                    executeBurnSequence(satellite, functions, t, neo, false);
                } else {
                    printf("\r\nSatellite not armed!\r\n");
                }
                break;

            case hash_str("fc"):
                if (parsed == 2) {
                    switch (hash_runtime(arg)) {
                        case hash_str("on"):
                            printf("\r\nTurning flight computer on...\r\n");
                            functions.c.flight_computer_on();
                            break;
                        case hash_str("off"):
                            printf("\r\nTurning flight computer off...\r\n");
                            functions.c.flight_computer_off();
                            break;
                        default:
                            printf("\r\nInvalid argument for fc command\r\n");
                    }
                }
                break;

            case hash_str("5v"):
                if (parsed == 2 && hash_runtime(arg) == hash_str("on")) {
                    printf("\r\nEnabling 5V...\r\n");
                    functions.c.five_volt_enable();
                }
                break;

            case hash_str("lidar"):
                printf("\r\nLiDAR Distance: %dmm\r\n", functions.c.lidar.getDistance());
                break;

            case hash_str("led"):
                if (parsed == 2) {
                    printf("\r\nSetting LED to %s...\r\n", arg);
                    switch (hash_runtime(arg)) {
                        case hash_str("red"):
                            neo.put_pixel(neo.urgb_u32(LED_RED.r, LED_RED.g, LED_RED.b));
                            break;
                        case hash_str("green"):
                            neo.put_pixel(neo.urgb_u32(LED_GREEN.r, LED_GREEN.g, LED_GREEN.b));
                            break;
                        case hash_str("yellow"):
                            neo.put_pixel(neo.urgb_u32(LED_YELLOW.r, LED_YELLOW.g, LED_YELLOW.b));
                            break;
                        case hash_str("purple"):
                            neo.put_pixel(neo.urgb_u32(LED_PURPLE.r, LED_PURPLE.g, LED_PURPLE.b));
                            break;
                        case hash_str("off"):
                            neo.put_pixel(neo.urgb_u32(LED_OFF.r, LED_OFF.g, LED_OFF.b));
                            break;
                        default:
                            printf("\r\nInvalid LED color\r\n");
                    }
                }
                break;

            case hash_str("faces"):
                if (parsed == 2) {
                    switch (hash_runtime(arg)) {
                        case hash_str("on"):
                            printf("\r\nTurning faces on...\r\n");
                            satellite.all_faces_on();
                            break;
                        case hash_str("off"):
                            printf("\r\nTurning faces off...\r\n");
                            satellite.all_faces_off();
                            break;
                        default:
                            printf("\r\nInvalid argument for faces command\r\n");
                    }
                }
                break;

            case hash_str("camera"):
                if (parsed == 2) {
                    switch (hash_runtime(arg)) {
                        case hash_str("on"):
                            printf("\r\nTurning camera on...\r\n");
                            satellite.camera_on();
                            break;
                        case hash_str("off"):
                            printf("\r\nTurning camera off...\r\n");
                            satellite.camera_off();
                            break;
                        default:
                            printf("\r\nInvalid argument for camera command\r\n");
                    }
                }
                break;

            default:
                printf("\r\nUnknown command. Type 'help' for available commands.\r\n");
        }
    }
};


#endif