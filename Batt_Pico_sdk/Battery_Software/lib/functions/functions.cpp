#include "functions.h"

satellite_functions::satellite_functions(pysquared& satellite):t(true, "[FUNCTIONS] "), c(satellite){
    t.debug_print("Functions initialized\n");
}

void satellite_functions::battery_manager() {
    // Get all readings with validation checks
    float board_temp = c.board_temp();
    float battery_temp = c.getThermocoupleTemp();
    
    // Battery monitor readings
    float battery_voltage = c.getBatteryBusVoltage();
    float draw_current = c.getBatteryCurrent();
    
    // Solar monitor readings
    float charge_voltage = c.getSolarBusVoltage();
    float charge_current = c.getSolarCurrent();

    // Validate readings and log
    bool valid_readings = true;
    
    if (battery_temp < -50 || battery_temp > 100) {
        t.debug_print("WARNING: Invalid battery temperature reading: " + to_string(battery_temp) + "C\n");
        valid_readings = false;
    }
    
    if (board_temp < -50 || board_temp > 100) {
        t.debug_print("WARNING: Invalid board temperature reading: " + to_string(board_temp) + "C\n");
        valid_readings = false;
    }
    
    if (battery_voltage < 0 || battery_voltage > 10) {
        t.debug_print("WARNING: Invalid battery voltage reading: " + to_string(battery_voltage) + "V\n");
        valid_readings = false;
    }

    // Log all readings
    t.debug_print("Board Temperature: " + to_string(board_temp) + "C\n");
    t.debug_print("Battery Temperature: " + to_string(battery_temp) + "C\n");
    t.debug_print("Battery Voltage: " + to_string(battery_voltage) + "V\n");
    t.debug_print("Charge Voltage: " + to_string(charge_voltage) + "V\n");
    t.debug_print("Draw Current: " + to_string(draw_current) + "mA\n");
    t.debug_print("Charge Current: " + to_string(charge_current) + "mA\n");

    // Only proceed with power mode changes if readings are valid
    if (valid_readings) {
        // Power mode setting based on battery voltage
        if (battery_voltage >= 7.4) {
            t.debug_print("Setting power mode to 3 (maximum)\n");
            c.pwr_mode = 3;
        } else if (battery_voltage >= 6.8) {
            t.debug_print("Setting power mode to 2 (normal)\n");
            c.pwr_mode = 2;
        } else if (battery_voltage >= 5.8) {
            t.debug_print("Setting power mode to 1 (low)\n");
            c.pwr_mode = 1;
        } else {
            t.debug_print("Setting power mode to 0 (critical)\n");
            c.pwr_mode = 0;
        }

        // Check charging status
        if (draw_current > charge_current && c.is_charging()) {
            t.debug_print("Battery is depleting... Battery is charging slower than system is drawing!\n");
            c.charging(true);
        }

        // Temperature management
        if (battery_temp < -10 && board_temp < -10) {
            t.debug_print("battery temperature is low, attempting to heat...\n");
            battery_heater();
        }
    } else {
        // If readings are invalid, default to a safe power mode
        t.debug_print("WARNING: Invalid readings detected, defaulting to low power mode\n");
        c.pwr_mode = 1;  // Or could go to critical (0) if you want to be more conservative
    }
}

void satellite_functions::battery_heater(){
    float battery_temp = c.thermocouple_temp();
    float board_temp = c.board_temp();
    int counter = 0;
    t.debug_print("Battery temperature before heating: " + to_string(battery_temp) + "C\n");
    try{
        c.heater_on();
        while(battery_temp < 0 && board_temp < 0 && counter < 100){
            sleep_ms(10);
            battery_temp = c.thermocouple_temp();
            counter++;
        }
        c.heater_off();
        t.debug_print("Battery temperature after heating: " + to_string(battery_temp) + "C\n");
    }
    catch(...){
        t.debug_print("Error within heater handling!\n");
        c.heater_off();
    }
    return;
}

bool satellite_functions::burn_handler(bool has_been_attempted){
    int max_allowed_burn_time=4;
    int counter = 0;
    int attempts = 0;
    if(has_been_attempted){
        max_allowed_burn_time = 5;
    }
    try{
        while(attempts < 5){
            counter = 0;
            c.burn_on(0.20 + (attempts*0.05));
            while(counter < (max_allowed_burn_time * 100)){
                sleep_ms(10);
                c.uart_receive_handler();
                if(c.burned){
                    break;
                }
                if(counter%100==0){
                    watchdog_update();
                }
                counter++;
            }
            c.burn_off();
            if(counter < (max_allowed_burn_time * 100)){
                return true;
            }
            else{
                sleep_ms(2000);
            }
            attempts++;
            t.debug_print("No confirmation of burn after attempt " + to_string(attempts) + "\n");
        }
        return false;
    }
    catch(...){
        t.debug_print("Error in burn sequence!\n");
        return false;
    }
    
}

void satellite_functions::short_hybernate() {
   t.debug_print("SHORT hybernation starting!\n");
   t.safe_sleep(SHORT_HIBERNATE_SECONDS * 1000);  // Convert seconds to milliseconds
   t.debug_print("SHORT hybernation complete\n");
}

void satellite_functions::long_hybernate() {
    t.debug_print("Long hybernation starting!\n");
    // Sleep for ~400 seconds (previous was 100 * 4000ms = 400s)
    uint32_t wake_time = /* TODO: Get current RTC time */ + LONG_HIBERNATE_SECONDS;
    t.sleep_until_rtc(wake_time);
}

void satellite_functions::handle_errors(){
    try{
        t.debug_print("number of errors in run so far: " + to_string(c.num_error()) + "\n");
    }
    catch(...){
        t.debug_print("Error collecting errors!\n");
    }
}