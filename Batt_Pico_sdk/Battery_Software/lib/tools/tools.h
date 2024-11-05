#ifndef _TOOLS_H
#define _TOOLS_H

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"  // Add this for watchdog_update()
#include "hardware/clocks.h"    // For clock functions


using namespace std;

class tools {
public:
    bool debug;
    const char* program;
    
    tools(bool print, const char* prog);
    void debug_print(const string message);
    void i2c_scan(i2c_inst_t *i2c);
    void safe_sleep(uint32_t ms);  // Add the safe_sleep declaration
    void sleep_until_rtc(uint32_t wake_time_seconds);
};

#endif