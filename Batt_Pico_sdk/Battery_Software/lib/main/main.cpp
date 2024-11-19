//Modified with Claude 3.5 Sonnet

#include "main.h"

void main_program(neopixel neo) {
    sleep_ms(10);
    tools t(true, "[MAIN] ");
    t.debug_print("I am in main!\n");
    neo.put_pixel(neo.urgb_u32(LED_GREEN.r, LED_GREEN.g, LED_GREEN.b));
    t.debug_print("Initializing Hardware...\n");

    // Initialize watchdog
    bool watchdog_reset = initializeWatchdog(t);

    // Initialize satellite and functions
    pysquared satellite(neo);
    satellite_functions functions(satellite);

    // Handle flash memory
    uint8_t data[1u<<8];
    handleFlashInitialization(satellite, t, watchdog_reset, data);
    
    // Check burn status
    BurnStatus burnStatus = checkBurnStatus(satellite, t, data);
    
    // Update boot count
    updateBootCount(satellite, t, data);

    // Execute burn sequence if conditions are met
    if (!satellite.burned && !burnStatus.previous_brownout && satellite.is_armed()) {
        executeBurnSequence(satellite, functions, t, neo, burnStatus.has_burned_before);
    }
     
    // Main operation loop
    while (true) {
        satellite.camera_off();
        sleep_ms(SLEEP_INTERVAL_MS);
        watchdog_update();
        satellite.all_faces_on();
        functions.battery_manager();
        t.debug_print("about to enter the main loop!\n");
        
        // runMainLoop(satellite, functions, t, neo);

        test_can(satellite, neo, functions);
    }
}

/* 
====================
Main Loop Operations
====================
*/

void runMainLoop(pysquared& satellite, satellite_functions& functions, 
                tools& t, neopixel& neo) {
    // uint8_t stuff[] = {0x05};
    while (true) {
        watchdog_update();
        functions.c.uart_receive_handler();
        
        switch (satellite.power_mode()) {
            case 0:
                critical_power_operations(t, functions);
                break;
            case 1:
                low_power_operations(t, neo, functions);
                break;
            case 2:
                normal_power_operations(t, neo, functions);
                break;
            case 3:
                maximum_power_operations(t, neo, functions);
                break;
        }
        satellite.check_reboot();
    }
}

void critical_power_operations(tools t, satellite_functions functions) {
    t.debug_print("Satellite is in critical power mode!\n");
    functions.c.flight_computer_on();
    // NOTE: in Critical Power we turn off the watchdog here
    functions.c.five_volt_disable();

    functions.c.uart_receive_handler();
    sleep_ms(SLEEP_INTERVAL_MS);
    functions.c.uart_receive_handler();
    sleep_ms(SLEEP_INTERVAL_MS);
    functions.c.flight_computer_off();
    functions.long_hybernate();
    functions.battery_manager();
    watchdog_update();
}

void low_power_operations(tools t, neopixel neo, satellite_functions functions) {
    t.debug_print("Satellite is in low power mode!\n");
    neo.put_pixel(neo.urgb_u32(LED_RED.r, LED_RED.g, LED_RED.b));
    functions.c.flight_computer_on();
    functions.c.five_volt_enable();
    functions.c.uart_receive_handler();
    
    for (int i = 0; i < 9; i++) {
        t.safe_sleep(SLEEP_INTERVAL_MS);
        functions.c.uart_receive_handler();
    }
    
    t.safe_sleep(SLEEP_INTERVAL_MS);
    functions.c.flight_computer_off();
    functions.short_hybernate();
    functions.battery_manager();
}

void normal_power_operations(tools t, neopixel neo, satellite_functions functions) {
    t.debug_print("Satellite is in normal power mode!\n");
    neo.put_pixel(neo.urgb_u32(LED_YELLOW.r, LED_YELLOW.g, LED_YELLOW.b));
    functions.c.flight_computer_on();
    functions.c.five_volt_enable();
    functions.battery_manager();
    functions.c.uart_receive_handler();
    t.debug_print("LiDAR Distance: " + to_string(functions.c.lidar.getDistance()) + "mm\n");
    watchdog_update();
    t.safe_sleep(SLEEP_INTERVAL_MS);
}

void maximum_power_operations(tools t, neopixel neo, satellite_functions functions) {
    t.debug_print("Satellite is in maximum power mode!\n");
    neo.put_pixel(neo.urgb_u32(LED_GREEN.r, LED_GREEN.g, LED_GREEN.b));
    functions.c.flight_computer_on();
    functions.c.five_volt_enable();
    functions.battery_manager();
    functions.c.uart_receive_handler();
    t.debug_print("LiDAR Distance: " + to_string(functions.c.lidar.getDistance()) + "mm\n");
    watchdog_update();
    t.safe_sleep(SLEEP_INTERVAL_MS);
}

/*
====================
Test Loops
====================
*/
void test_can(pysquared satellite, neopixel neo, satellite_functions functions) {
    tools t(true, "[CAN TEST] ");
    t.debug_print("Starting UART test sequence...\n");

    functions.c.flight_computer_on();
    functions.c.five_volt_enable();
    
    neo.put_pixel(neo.urgb_u32(LED_YELLOW.r, LED_YELLOW.g, LED_YELLOW.b));
    neo.put_pixel(neo.urgb_u32(0x00,0x00,0x00));
    
    /*
    // Prepare the CAN message - using 8 bytes max for standard CAN
    struct can_frame frame;
    frame.can_id = 0x123;    // Arbitrary CAN ID
    frame.can_dlc = 7;       // Length of "Testing"
    
    // Message that fits in 8 bytes
    const char* message = "Testing";
    memcpy(frame.data, message, frame.can_dlc);
    
    t.debug_print("Beginning transmission loop...\n");
    
    while (true) {
        MCP2515::ERROR result = satellite.can_bus.sendMessage(&frame);
        
        if (result == MCP2515::ERROR_OK) {
            t.debug_print("CAN message sent successfully\n");
            neo.put_pixel(neo.urgb_u32(LED_GREEN.r, LED_GREEN.g, LED_GREEN.b));
        } else {
            t.debug_print("Error sending message: " + std::to_string(static_cast<int>(result)) + "\n");
            neo.put_pixel(neo.urgb_u32(LED_RED.r, LED_RED.g, LED_RED.b));
        }
        
        sleep_ms(1000);  // Wait for 1 second before next transmission
        watchdog_update();  // Keep the watchdog happy
    }
    */
   while (true) {
       functions.c.uart_receive_handler();
       t.debug_print("Checked!\n");
       watchdog_update();
       sleep_ms(100);
       // t.safe_sleep(5000);
   }
}


/*
====================
Helper Functions
====================
*/

bool initializeWatchdog(tools& t) {
    bool watchdog_reset = false;
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
    t.debug_print("Internal Watchdog enabled\n");
    if (watchdog_caused_reboot()) {
        t.debug_print("Rebooted by Watchdog!\n");
        watchdog_reset = true;
    } else {
        t.debug_print("Clean boot\n");
    }
    return watchdog_reset;
}

void handleFlashInitialization(pysquared& satellite, tools& t, bool watchdog_reset, uint8_t* data) {
    // satellite.flash_init();    
    // satellite.flash_read(data, 0);
    
    if (watchdog_reset) {
        t.debug_print("setting watchdog tracker!\n");
        satellite.bit_set(STATUS_REG, WATCHDOG_BIT, true);
    } else {
        t.debug_print("unsetting watchdog tracker!\n");
        satellite.bit_set(STATUS_REG, WATCHDOG_BIT, false);
    }
}

BurnStatus checkBurnStatus(pysquared& satellite, tools& t, uint8_t* data) {
    BurnStatus status = {false, false};
    
    if ((data[STATUS_REG] & (1 << PRIOR_BURN_ATTEMPT)) == (1 << PRIOR_BURN_ATTEMPT)) {
        t.debug_print("memory indicates a previously failed burn attempt!\n");
        status.has_burned_before = true;
    } else {
        t.debug_print("Memory indicates this is the first burn attempt ever! good luck...\n");
        satellite.bit_set(STATUS_REG, PRIOR_BURN_ATTEMPT, true);
    }

    if ((data[STATUS_REG] & (1 << BURNED_BIT)) == (1 << BURNED_BIT)) {
        t.debug_print("memory indicates a previous burn was successful!\n");
        satellite.burned = true;
    } else {
        if ((data[STATUS_REG] & (1 << BROWNOUT_BIT)) == (1 << BROWNOUT_BIT)) {
            t.debug_print("memory indicates a brownout previously and no successful burn attempt. will not burn...\n");
            status.previous_brownout = true;
        } else {
            satellite.bit_set(STATUS_REG, BROWNOUT_BIT, true);
        }
    }
    
    return status;
}

void updateBootCount(pysquared& satellite, tools& t, uint8_t* data) {
    data[BOOT_REG]++;
    satellite.reg_set(BOOT_REG, data[BOOT_REG]);
    // satellite.flash_update();
    // satellite.flash_read(data, 0);
    t.debug_print("updated boot count: " + to_string(data[BOOT_REG]) + "\n");
    t.debug_print("updated status reg: " + to_string(data[STATUS_REG]) + "\n");
}

bool executeBurnSequence(pysquared& satellite, satellite_functions& functions, 
                        tools& t, neopixel& neo, bool has_burned_before) {
    uint counter = 0;
    while (counter < LOITER_TIME_SECONDS) {
        t.debug_print("Commencing burnwire in " + to_string(LOITER_TIME_SECONDS-counter) + "seconds...\n");
        neo.put_pixel(neo.urgb_u32(LED_PURPLE.r, LED_PURPLE.g, LED_PURPLE.b));
        sleep_ms(BLINK_INTERVAL_MS);
        neo.put_pixel(neo.urgb_u32(LED_OFF.r, LED_OFF.g, LED_OFF.b));
        sleep_ms(BLINK_INTERVAL_MS);
        watchdog_update();
        counter++;
    }
    
    if (functions.burn_handler(has_burned_before)) {
        satellite.arm(false);
        satellite.bit_set(STATUS_REG, BURNED_BIT, true);
        satellite.bit_set(STATUS_REG, BROWNOUT_BIT, false);
        // satellite.flash_update();
        t.debug_print("Flash updated to reflect successful burn and disarmed status!\n");
        return true;
    }
    return false;
}

