#ifndef TOOLS_H
#define TOOLS_H

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

using namespace std;

class pysquared;
class satellite_functions;
class neopixel;

class tools {
private:
    static constexpr size_t COMMAND_BUFFER_SIZE = 256;
    static constexpr size_t MAX_COMMAND_LENGTH = 32;
    
    char input_buffer[COMMAND_BUFFER_SIZE];
    size_t buffer_pos;  // Changed from volatile int to size_t
    volatile bool line_received;
    const char* debug_prefix;
    
    // Helper functions for command processing
    constexpr static uint32_t hash_str(const char* str);
    uint32_t hash_runtime(const char* str);
    void execute_command(pysquared& satellite, satellite_functions& functions, neopixel& neo);

public:
    bool debug;
    const char* program;

    // Constructor
    tools(bool print, const char* prog);

    // Existing tools methods
    void debug_print(const string message);
    void i2c_scan(i2c_inst_t *i2c);
    void safe_sleep(uint32_t ms);
    void sleep_until_rtc(uint32_t wake_time_seconds);

    // Command system methods
    void init_command_system();
    void process_input(pysquared& satellite, satellite_functions& functions, neopixel& neo);
    void print_help();
    void test_burn_wire(pysquared& satellite, satellite_functions& functions, float duty_cycle);
};

#endif