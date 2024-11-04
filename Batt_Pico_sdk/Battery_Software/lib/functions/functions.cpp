#include "functions.h"

satellite_functions::satellite_functions(pysquared& satellite):t(true, "[FUNCTIONS] "), c(satellite){
    t.debug_print("Functions initialized\n");
}

void satellite_functions::battery_manager(){
    float board_temp = c.board_temp();
    float battery_temp = c.thermocouple_temp();
    float battery_voltage = c.battery_voltage();
    float draw_current = c.draw_current();
    float charge_voltage = c.charge_voltage();
    float charge_current = c.charge_current();
    t.debug_print("Board Temperature: " + to_string(board_temp) + "C\n");
    t.debug_print("Battery Temperature: " + to_string(battery_temp) + "C\n");
    t.debug_print("Battery Voltage: " + to_string(battery_voltage) + "V\n");
    t.debug_print("Charge Voltage: " + to_string(charge_voltage) + "V\n");
    t.debug_print("Draw Current: " + to_string(draw_current) + "mA\n");
    t.debug_print("Charge Current: " + to_string(charge_current) + "mA\n");
    if(battery_voltage >= 7.4){c.pwr_mode=3;}
    else if(battery_voltage < 7.4 && battery_voltage >= 6.8){c.pwr_mode=2;}
    else if(battery_voltage < 6.8 && battery_voltage >= 6.4){c.pwr_mode=1;}
    else if(battery_voltage < 6.4){c.pwr_mode=0;}
    if(draw_current > charge_current && c.is_charging()){
        t.debug_print("Battery is depleting... Battery is charging slower than system is drawing!\n");
        c.charging(true);
    }
    if(battery_temp < -10 && board_temp < -10){
        t.debug_print("battery temperature is low, attempting to heat...\n");
        battery_heater();
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

void satellite_functions::long_hybernate(){
    t.debug_print("Long hybernation starting!\n");
    for(int i = 0; i < 100; i++){
        sleep_ms(2000);
        watchdog_update();
        sleep_ms(2000);
    }
}

void satellite_functions::short_hybernate(){
    t.debug_print("short hybernation starting!\n");
    for(int i = 0; i < 10; i++){
        sleep_ms(2000);
        watchdog_update();
        sleep_ms(2000);
    }
}

void satellite_functions::handle_errors(){
    try{
        t.debug_print("number of errors in run so far: " + to_string(c.num_error()) + "\n");
    }
    catch(...){
        t.debug_print("Error collecting errors!\n");
    }
}