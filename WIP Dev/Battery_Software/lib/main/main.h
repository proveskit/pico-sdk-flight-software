#ifndef _MAIN_H
#define _MAIN_H
#include <stdio.h>
#include <neopixel/neopixel.h>
#include <tools/tools.h>
#include <pysquared/pysquared.h>
#include <functions/functions.h>
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <chrono>
#include <iostream>
#include <string>

void main_program(neopixel neo);

void debug_print(tools t, satellite_functions functions, String statement);
void boot_sequence(tools t, satellite_functions functions);
void critical_power_operations(tools t, satellite_functions functions);
void low_power_operations(tools t, neopixel neo, satellite_functions functions);
void normal_power_operations(tools t, neopixel neo, satellite_functions functions);
void maximum_power_operations(tools t, neopixel neo, satellite_functions functions);

#endif