#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H
#include <stdio.h>
#include <neopixel/neopixel.h>
#include <tools/tools.h>
#include <pysquared/pysquared.h>
#include <device_drivers/MCP25625.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"

class satellite_functions{
    private:
    tools t;

    //Hibernate times
    static constexpr uint32_t SHORT_HIBERNATE_SECONDS = 10;  // 10 seconds
    static constexpr uint32_t LONG_HIBERNATE_SECONDS = 400;  // ~6.5 minutes

    public:
    satellite_functions(pysquared& satellite);
    void battery_manager();
    void battery_heater();
    bool burn_handler(bool has_been_attempted);
    void long_hybernate();
    void short_hybernate();
    void handle_errors();
    pysquared& c;

};
#endif
