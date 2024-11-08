#include "tools.h"
#include "../pysquared/pysquared.h"
#include "../functions/functions.h"
#include "../neopixel/neopixel.h"

#include "pico/stdlib.h"


// Define LED colors
struct LED_Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static const LED_Color LED_RED = {255, 0, 0};
static const LED_Color LED_GREEN = {0, 255, 0};
static const LED_Color LED_YELLOW = {255, 255, 0};
static const LED_Color LED_PURPLE = {128, 0, 128};
static const LED_Color LED_OFF = {0, 0, 0};

tools::tools(bool print, const char* prog) 
    : buffer_pos(0)
    , line_received(false)
    , debug(print)
    , program(prog)
{
    // Keep constructor minimal
    printf("Basic tools construction complete\n");
}

void tools::debug_print(const string message){
    if(debug){
        uint32_t sec_since_boot = to_ms_since_boot(get_absolute_time())/1000;
        uint32_t min_since_boot = sec_since_boot/60;
        uint secs = sec_since_boot%60;
        uint mins = min_since_boot%60;
        uint hours = min_since_boot/60;
        printf("%02d:%02d:%02d ",hours,mins,secs);
        printf(program);
        printf(message.c_str());
    }
}
void tools::i2c_scan(i2c_inst_t *i2c){
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        ret = i2c_read_timeout_us(i2c, addr, &rxdata, 1, false, 100000);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

void tools::safe_sleep(uint32_t ms) {
    const uint32_t WATCHDOG_INTERVAL = 1000;  // Update every second
    while (ms > 0) {
        uint32_t sleep_time = (ms < WATCHDOG_INTERVAL) ? ms : WATCHDOG_INTERVAL;
        sleep_ms(sleep_time);
        watchdog_update();
        ms -= sleep_time;
        if (debug) {
            debug_print("Watchdog updated during sleep...\n");
        }
    }
}

void tools::sleep_until_rtc(uint32_t wake_time_seconds) {
    if (debug) {
        debug_print("Preparing for RTC-based deep sleep\n");
    }
    
    debug_print("RTC deep sleep not yet implemented\n");
    safe_sleep(wake_time_seconds * 1000);  // Fall back to regular sleep
}


constexpr uint32_t tools::hash_str(const char* str) {
    uint32_t hash = 5381;
    while (*str) {
        hash = ((hash << 5) + hash) + *str++;
    }
    return hash;
}

uint32_t tools::hash_runtime(const char* str) {
    uint32_t hash = 5381;
    while (*str) {
        hash = ((hash << 5) + hash) + *str++;
    }
    return hash;
}

void tools::init_command_system() {
    printf("Command system initializing...\n");
    buffer_pos = 0;
    line_received = false;
    
    debug_print("=========================\n");
    debug_print("PySqaured Test Console\n");
    debug_print("Type 'help' for available commands\n");
    debug_print("=========================\n");
    printf("> ");
    fflush(stdout);
}

void tools::process_input(pysquared& satellite, satellite_functions& functions, neopixel& neo) {
    try {
        int c = getchar_timeout_us(0);  // Non-blocking getchar
        
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\r' || c == '\n') {
                if (buffer_pos > 0) {
                    printf("\n");
                    input_buffer[buffer_pos] = '\0';
                    line_received = true;
                    buffer_pos = 0;
                }
            }
            else if (c == 0x7F || c == 0x08) {  // Backspace
                if (buffer_pos > 0) {
                    buffer_pos--;
                    printf("\b \b");
                }
            }
            else if (c >= 32 && c <= 126 && buffer_pos < (COMMAND_BUFFER_SIZE - 1)) {
                input_buffer[buffer_pos++] = static_cast<char>(c);
                putchar(c);
                fflush(stdout);  // Make sure character is displayed
            }
        }

        if (line_received) {
            execute_command(satellite, functions, neo);
            line_received = false;
            printf("\n> ");
            fflush(stdout);
        }
    } catch(...) {
        printf("Error in process_input!\n");
    }
}

void tools::test_burn_wire(pysquared& satellite, satellite_functions& functions, float duty_cycle) {
    if (!satellite.is_armed()) {
        debug_print("ERROR: Satellite must be armed to test burn wire!\n");
        return;
    }

    if (duty_cycle < 0.0 || duty_cycle > 1.0) {
        debug_print("ERROR: Duty cycle must be between 0.0 and 1.0\n");
        return;
    }

    debug_print("Starting burn wire test sequence\n");
    debug_print("WARNING: This will activate the burn wire at " + to_string(duty_cycle * 100) + "% duty cycle!\n");
    debug_print("Press 'q' to abort, any other key to continue...\n");
    
    // Wait for input with timeout
    int c;
    do {
        c = getchar_timeout_us(100000);  // 100ms timeout
    } while (c == PICO_ERROR_TIMEOUT);
    
    if (c == 'q' || c == 'Q') {
        debug_print("Burn wire test aborted.\n");
        return;
    }

    try {
        debug_print("Testing burn wire at " + to_string(duty_cycle * 100) + "% duty cycle for 2 seconds...\n");
        functions.c.burn_on(duty_cycle);
        
        for (int i = 0; i < 20; i++) {
            sleep_ms(100);
            watchdog_update();
            
            if (functions.c.burned) {
                debug_print("Burn detected!\n");
                break;
            }
        }
        
        functions.c.burn_off();
        debug_print("Burn wire test complete.\n");
        
    } catch(...) {
        functions.c.burn_off();
        debug_print("Error during burn wire test!\n");
    }
}

void tools::print_help() {
    printf("\nAvailable Commands:\n");
    printf("help            - Show this help message\n");
    printf("status          - Show satellite status\n");
    printf("arm <on/off>    - Arm or disarm the satellite\n");
    printf("battery         - Run battery manager\n");
    printf("burn (Not Working) - Test burn sequence\n");
    printf("fc <on/off>     - Turn flight computer on/off\n");
    printf("5v <on/off>     - Toggle 5V power\n");
    printf("lidar           - Get LiDAR reading\n");
    printf("led <color>     - Set LED (red/green/yellow/purple/off)\n");
    printf("faces <on/off>  - Control all faces\n");
    printf("camera <on/off> - Control camera\n");
}

void tools::execute_command(pysquared& satellite, satellite_functions& functions, neopixel& neo) {
    char cmd[MAX_COMMAND_LENGTH];
    char arg[MAX_COMMAND_LENGTH];
    char arg2[MAX_COMMAND_LENGTH];
    int parsed = sscanf(input_buffer, "%s %s %s", cmd, arg, arg2);

    if (parsed <= 0) return;

    switch (hash_runtime(cmd)) {
        case hash_str("help"):
            print_help();
            break;

        case hash_str("status"):
            debug_print("Power Mode: " + to_string(satellite.power_mode()));
            debug_print("\nBurned: " + string(satellite.burned ? "Yes" : "No"));
            debug_print("\nArmed: " + string(satellite.is_armed() ? "Yes" : "No"));
            break;

        case hash_str("arm"):
            if (parsed == 2) {
                if (strcmp(arg, "on") == 0) {
                    debug_print("Arming satellite...\n");
                    satellite.arm(true);
                    debug_print("Satellite armed successfully.\n");
                }
                else if (strcmp(arg, "off") == 0) {
                    debug_print("Disarming satellite...\n");
                    satellite.arm(false);
                    debug_print("Satellite disarmed successfully.\n");
                }
                else {
                    debug_print("Invalid argument. Use 'arm on' or 'arm off'\n");
                }
            } else {
                debug_print("Usage: arm <on/off>\n");
            }
            break;

        case hash_str("battery"):
            debug_print("Running battery manager...");
            functions.battery_manager();
            break;

        case hash_str("burn"):
            if (parsed >= 2 && strcmp(arg, "test") == 0) {
                float duty_cycle = 0.20;  // Default duty cycle if none specified
                if (parsed == 3) {
                    // Convert percentage to decimal (e.g., 20 becomes 0.20)
                    duty_cycle = atof(arg2) / 100.0;
                }
                test_burn_wire(satellite, functions, duty_cycle);
            } else {
                debug_print("Usage: burn test [duty_cycle]\n");
                debug_print("  duty_cycle: percentage between 0-100 (default: 20)\n");
                debug_print("  Example: 'burn test 25' for 25% duty cycle\n");
            }
            break;

        case hash_str("fc"):
            if (parsed == 2) {
                switch (hash_runtime(arg)) {
                    case hash_str("on"):
                        debug_print("Turning flight computer on...");
                        functions.c.flight_computer_on();
                        break;
                    case hash_str("off"):
                        debug_print("Turning flight computer off...");
                        functions.c.flight_computer_off();
                        break;
                    default:
                        debug_print("Invalid argument for fc command");
                }
            }
            break;

        case hash_str("5v"):
            if (parsed == 2 && hash_runtime(arg) == hash_str("on")) {
                debug_print("Enabling 5V...");
                functions.c.five_volt_enable();
            }
            break;

        case hash_str("lidar"):
            debug_print("LiDAR Distance: " + to_string(functions.c.lidar.getDistance()) + "mm");
            break;

        case hash_str("led"):
            if (parsed == 2) {
                debug_print("Setting LED to " + string(arg) + "...");
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
                        debug_print("Invalid LED color");
                }
            }
            break;

        case hash_str("faces"):
            if (parsed == 2) {
                switch (hash_runtime(arg)) {
                    case hash_str("on"):
                        debug_print("Turning faces on...");
                        satellite.all_faces_on();
                        break;
                    case hash_str("off"):
                        debug_print("Turning faces off...");
                        satellite.all_faces_off();
                        break;
                    default:
                        debug_print("Invalid argument for faces command");
                }
            }
            break;

        case hash_str("camera"):
            if (parsed == 2) {
                switch (hash_runtime(arg)) {
                    case hash_str("on"):
                        debug_print("Turning camera on...");
                        satellite.camera_on();
                        break;
                    case hash_str("off"):
                        debug_print("Turning camera off...");
                        satellite.camera_off();
                        break;
                    default:
                        debug_print("Invalid argument for camera command");
                }
            }
            break;

        default:
            debug_print("Unknown command. Type 'help' for available commands.");
    }
}