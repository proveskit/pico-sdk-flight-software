#include "pysquared.h"
                                                      //instantiate tools class in pysquared
pysquared::pysquared(neopixel neo) : 
    t(true, "[PYSQUARED] "),
    battery_power(i2c0, 0x40),
    solar_power(i2c0, 0x44),
    internal_temp(i2c0, 0x4f),
    lidar(i2c0, 0x29),
    led_driver(i2c0, 0x56),
    adc(i2c0, 0x48),
    can_bus(spi0, spi0_cs0_pin)
{
    /*
        Initialize hardware core to operations on satellite.
    */
    try {
        pwr_mode=2;
        /*
            GPIO init
        */
        gpio_init(fc_reset_pin);
        gpio_set_dir(fc_reset_pin, GPIO_OUT);
        gpio_init(spi0_cs0_pin);
        gpio_set_dir(spi0_cs0_pin, GPIO_OUT);
        gpio_init(rf_enable_pin);
        gpio_set_dir(rf_enable_pin, GPIO_OUT);
        gpio_init(vbus_reset_pin);
        gpio_set_dir(vbus_reset_pin, GPIO_OUT);
        gpio_init(relay_pin);
        gpio_set_dir(relay_pin, GPIO_OUT);
        //gpio_init(spi1_cs0_pin);
        //gpio_set_dir(spi1_cs0_pin, GPIO_OUT);
        gpio_init(d0_pin);
        gpio_set_dir(d0_pin, GPIO_IN);
        gpio_init(is_charge_pin);
        gpio_set_dir(is_charge_pin, GPIO_IN);
        gpio_init(jetson_enable_pin);
        gpio_set_dir(jetson_enable_pin, GPIO_OUT);
        gpio_init(five_volt_enable_pin);
        gpio_set_dir(five_volt_enable_pin, GPIO_OUT);
        t.debug_print("enable burn pin not init");
        gpio_init(enable_burn_pin);
        gpio_set_dir(enable_burn_pin, GPIO_OUT);
        t.debug_print("enable burn pin init");
        t.debug_print("GPIO Pins Initialized!\n");
        /*
            PWM init
        */
        /*gpio_set_function(enable_burn_pin, GPIO_FUNC_PWM);*/
        gpio_set_function(enable_heater_pin, GPIO_FUNC_PWM);
        /*burn_slice = pwm_gpio_to_slice_num(enable_burn_pin);*/
        heater_slice = pwm_gpio_to_slice_num(enable_heater_pin);
        t.debug_print("PWM Pins Initialized!\n");
        /*
            I2C init
        */
        i2c_init(i2c0, 400*1000);
        gpio_set_function(i2c_sda0_pin, GPIO_FUNC_I2C);
        gpio_set_function(i2c_scl0_pin, GPIO_FUNC_I2C);
        t.i2c_scan(i2c0);
        i2c_init(i2c1, 400*1000);
        gpio_set_function(i2c_sda1_pin, GPIO_FUNC_I2C);
        gpio_set_function(i2c_scl1_pin, GPIO_FUNC_I2C);
        t.i2c_scan(i2c1);
        t.debug_print("I2C Bus Initialized!\n");
        /*
            UART init
        */
        uart_init(uart0,2400);
        gpio_set_function(uart_tx, GPIO_FUNC_UART);
        gpio_set_function(uart_rx, GPIO_FUNC_UART);
        uart_set_baudrate(uart0,BAUD_RATE);
        uart_set_hw_flow(uart0, false, false);
        uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart0, true);
        t.debug_print("UART Bus Initialized!\n");
        /*
            SPI init
        */
        spi_init(spi0, 500 * 1000);
        gpio_set_function(spi0_miso_pin, GPIO_FUNC_SPI);
        gpio_set_function(spi0_sck_pin, GPIO_FUNC_SPI);
        gpio_set_function(spi0_mosi_pin, GPIO_FUNC_SPI);
        //spi_init(spi1, 500 * 1000);
        //gpio_set_function(spi1_miso_pin, GPIO_FUNC_SPI);
        //gpio_set_function(spi1_sck_pin, GPIO_FUNC_SPI);
        //gpio_set_function(spi1_mosi_pin, GPIO_FUNC_SPI);
        t.debug_print("SPI Bus Initialized!\n");

        // LED Driver init
        try {
            led_driver.configure();
            led_driver_initialized = true;
            all_faces_on();
            camera_on();
            t.debug_print("LED Driver Initialized!\n");
        } catch(...) {
            led_driver_initialized = false;
            t.debug_print("ERROR initializing LED Driver!\n");
            error_count++;
        }

        // ADC init
        try { 
            adc.configure();
            adc_initialized = true;
            t.debug_print("Thermocouple ADC Initialized!\n");
        } catch(...) {
            adc_initialized = false;
            t.debug_print("ERROR initializing Thermocouple ADC!\n");
            error_count++;
        }

        // Battery Monitor init
        try {
            battery_power.configure();
            battery_monitor_initialized = true;
            t.debug_print("Battery Power Monitor Initialized!\n");
        } catch(...) {
            battery_monitor_initialized = false;
            t.debug_print("ERROR initializing Battery Power Monitor!\n");
            error_count++;
        }

        // Solar Monitor init
        try {
            solar_power.configure();
            solar_monitor_initialized = true;
            t.debug_print("Solar Power Monitor Initialized!\n");
        } catch(...) {
            solar_monitor_initialized = false;
            t.debug_print("ERROR initializing Solar Power Monitor!\n");
            error_count++;
        }

        // Initialize CAN bus
        /*
        BEGONE CAN BUS
        */
        
    } catch(...){
        t.debug_print("ERROR Initializing Hardware: \n");
        error_count++;
    }

}

// Status getter implementations
bool pysquared::is_lidar_ready() const {
    return lidar_initialized;
}

bool pysquared::is_led_driver_ready() const {
    return led_driver_initialized;
}

bool pysquared::is_adc_ready() const {
    return adc_initialized;
}

bool pysquared::is_battery_monitor_ready() const {
    return battery_monitor_initialized;
}

bool pysquared::is_solar_monitor_ready() const {
    return solar_monitor_initialized;
}

// Wrapper implementations
float pysquared::getLidarDistance() {
    if (!lidar_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized LiDAR\n");
        return -1.0f;
    }
    return lidar.getDistance();
}

void pysquared::configureLEDs() {  // Changed return type from bool to void
    if (!led_driver_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized LED Driver\n");
        return;
    }
    led_driver.configure();
}

float pysquared::getThermocoupleTemp() {
    if (!adc_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized ADC\n");
        return -999.0f;
    }
    return thermocouple_temp();
}

float pysquared::getBatteryBusVoltage() {
    if (!battery_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Battery Monitor\n");
        return -1.0f;
    }
    return battery_power.readBusVoltage();
}

float pysquared::getBatteryShuntVoltage() {
    if (!battery_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Battery Monitor\n");
        return -1.0f;
    }
    return battery_power.readShuntVoltage();
}

float pysquared::getBatteryCurrent() {
    if (!battery_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Battery Monitor\n");
        return -1.0f;
    }
    return battery_power.readCurrent();
}

float pysquared::getSolarBusVoltage() {
    if (!solar_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Solar Monitor\n");
        return -1.0f;
    }
    return solar_power.readBusVoltage();
}

float pysquared::getSolarShuntVoltage() {
    if (!solar_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Solar Monitor\n");
        return -1.0f;
    }
    return solar_power.readShuntVoltage();
}

float pysquared::getSolarCurrent() {
    if (!solar_monitor_initialized) {
        t.debug_print("WARNING: Attempt to use uninitialized Solar Monitor\n");
        return -1.0f;
    }
    return solar_power.readCurrent();
}

// MRAM Command Codes
#define MRAM_CMD_WREN  0x06  // Write Enable
#define MRAM_CMD_WRDI  0x04  // Write Disable
#define MRAM_CMD_RDSR  0x05  // Read Status Register
#define MRAM_CMD_WRSR  0x01  // Write Status Register
#define MRAM_CMD_READ  0x03  // Read Data
#define MRAM_CMD_WRITE 0x02  // Write Data

// Initialize the MRAM memory
void pysquared::mram_init() {
    try {
        t.debug_print("Initializing MRAM...\n");
        uint8_t data[256];
        uint8_t new_data[256];
        
        // Read first page to check initialization
        mram_read(data, 0);
        
        if(data[0] != 0) {
            t.debug_print("First time MRAM initialization\n");
            
            // Initialize with zeros
            for(int i = 0; i < 256; i++) {
                new_data[i] = 0xFF;
            }
            
            // Write zeros to both primary and backup pages
            mram_write(new_data, 0, 256);  // Primary page
            mram_write(new_data, 256, 256);  // Backup page
            
            t.debug_print("Testing initialization!\n");
            mram_read(data, 0);
            for(int i = 0; i < 256; i++) {
                if(data[i] != new_data[i]) {
                    t.debug_print("init failed on: " + to_string(i) + "; data[" + to_string(i) + "] = " + to_string(data[i]) + "\n");
                }
            }
            t.debug_print("MRAM initialized!\n");
        } else {
            t.debug_print("MRAM already initialized! Checking redundancy...\n");
            mram_read(new_data, 256);  // Read backup page
            for(int i = 0; i < 256; i++) {
                if(data[i] != new_data[i]) {
                    t.debug_print("memory failure on register: " + to_string(i) + "\n");
                    trust_memory = false;
                }
            }
            if(trust_memory) {
                t.debug_print("Memory is intact!\n");
            } else {
                t.debug_print("Memory cannot be trusted! Updating backup...\n");
                mram_write(data, 256, 256);  // Update backup page
            }
        }
    } catch(...) {
        t.debug_print("error initializing MRAM!\n");
        error_count++;
    }
}

// Read data from MRAM
void pysquared::mram_read(uint8_t *data, uint16_t address) {
    try {
        uint8_t cmd = MRAM_CMD_READ;
        uint8_t addr[3] = {
            (uint8_t)((address >> 16) & 0xFF),
            (uint8_t)((address >> 8) & 0xFF),
            (uint8_t)(address & 0xFF)
        };
        
        gpio_put(spi0_cs0_pin, 0);  // CS low
        
        // Send read command
        spi_write_blocking(spi0, &cmd, 1);
        // Send address
        spi_write_blocking(spi0, addr, 3);
        // Read data
        spi_read_blocking(spi0, 0, data, 256);
        
        gpio_put(spi0_cs0_pin, 1);  // CS high
        
        // Update memory variable
        for(int i = 0; i < 256; i++) {
            nvm_memory[i] = data[i];
        }
    } catch(...) {
        t.debug_print("error reading from MRAM!\n");
        error_count++;
    }
}

// Write data to MRAM
void pysquared::mram_write(const uint8_t *data, uint16_t address, uint16_t length) {
    try {
        // Enable write operations
        uint8_t wren_cmd = MRAM_CMD_WREN;
        gpio_put(spi0_cs0_pin, 0);
        spi_write_blocking(spi0, &wren_cmd, 1);
        gpio_put(spi0_cs0_pin, 1);
        
        // Write command and address
        uint8_t write_cmd = MRAM_CMD_WRITE;
        uint8_t addr[3] = {
            (uint8_t)((address >> 16) & 0xFF),
            (uint8_t)((address >> 8) & 0xFF),
            (uint8_t)(address & 0xFF)
        };
        
        gpio_put(spi0_cs0_pin, 0);
        spi_write_blocking(spi0, &write_cmd, 1);
        spi_write_blocking(spi0, addr, 3);
        
        // Write data
        spi_write_blocking(spi0, data, length);
        gpio_put(spi0_cs0_pin, 1);
        
        // Update memory variable
        for(int i = 0; i < length; i++) {
            nvm_memory[i] = data[i];
        }
    } catch(...) {
        t.debug_print("error writing to MRAM!\n");
        error_count++;
    }
}

// Update variable in memory
void pysquared::mram_variable_update(const uint8_t *data) {
    try {
        for(int i = 0; i < 256; i++) {
            nvm_memory[i] = data[i];
        }
    } catch(...) {
        t.debug_print("Failed to update variable for MRAM!\n");
        error_count++;
    }
}

// Read variable from memory
void pysquared::mram_variable_read(uint8_t *data) {
    try {
        for(int i = 0; i < 256; i++) {
            data[i] = nvm_memory[i];
        }
    } catch(...) {
        t.debug_print("failed to read from memory variable!\n");
        error_count++;
    }
}

/* WE NEED TO LOOK INTO THIS D;
void pysquared::flash_update(){
    try{
        t.debug_print("writing to flash...\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase((500*1024), FLASH_SECTOR_SIZE);
        flash_range_program((500*1024), nvm_memory, FLASH_PAGE_SIZE);
        flash_range_program((500*1024+256), nvm_memory, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        t.debug_print("Flash Updated!\n");
    }
    catch(...){
        t.debug_print("error writing to flag!\n");
        error_count++;
    }
}
*/

// Set register value
void pysquared::reg_set(const uint8_t reg, const uint8_t val) {
    try {
        nvm_memory[reg] = val;
        // Write to both primary and backup pages
        mram_write(&val, reg, 1);
        mram_write(&val, reg + 256, 1);
    } catch(...) {
        t.debug_print("ERROR Failed to update register!\n");
        error_count++;
    }
}

// Set bit in register
void pysquared::bit_set(const uint8_t reg, const uint8_t bit, bool state) {
    try {
        uint8_t data[256];
        mram_variable_read(data);
        if(state) {
            data[reg] |= (0x01 << bit);
        } else {
            uint8_t temp = 0xFF << (bit + 1);
            for(int i = 0; i < bit; i++) {
                temp = temp + (1u << i);
            }
            data[reg] &= temp;
        }
        reg_set(reg, data[reg]);
    } catch(...) {
        t.debug_print("ERROR with updating a NVM bit!\n");
        error_count++;
    }
}

bool pysquared::init_can_bus() {
    t.debug_print("[CAN] Starting initialization...\n");
    sleep_ms(100);  // Give hardware time to stabilize
    
    // Step 1: Reset
    t.debug_print("[CAN] Attempting reset...\n");
    MCP2515::ERROR result = can_bus.reset();
    if (result != MCP2515::ERROR_OK) {
        t.debug_print("[CAN] Reset failed with code: " + std::to_string(static_cast<int>(result)) + "\n");
        return false;
    }
    sleep_ms(100);  // Give time for reset to complete
    
    // Step 2: Set Configuration mode
    t.debug_print("[CAN] Setting config mode...\n");
    result = can_bus.setConfigMode();
    if (result != MCP2515::ERROR_OK) {
        t.debug_print("[CAN] Failed to enter config mode\n");
        return false;
    }
    sleep_ms(10);
    
    // Step 3: Set Bit Rate (assuming 16MHz clock)
    t.debug_print("[CAN] Setting bit rate...\n");
    result = can_bus.setBitrate(CAN_500KBPS, MCP_16MHZ);
    if (result != MCP2515::ERROR_OK) {
        t.debug_print("[CAN] Failed to set bit rate\n");
        return false;
    }
    
    // Step 4: Set Normal Mode
    t.debug_print("[CAN] Setting normal mode...\n");
    result = can_bus.setNormalMode();
    if (result != MCP2515::ERROR_OK) {
        t.debug_print("[CAN] Failed to enter normal mode\n");
        return false;
    }
    
    // Check status
    uint8_t status = can_bus.getStatus();
    t.debug_print("[CAN] Final status: 0x" + std::to_string(status) + "\n");
    
    t.debug_print("[CAN] Initialization completed successfully\n");
    return true;
}

void pysquared::bus_reset(){
    t.debug_print("Attempting to reset bus voltage...\n");  
    try{
        bit_set(1,4,false);
        //WE NEED TO LOOK INTO THIS OH NO D;
        //flash_update();
        for(int i = 10; i > 0; i --){
            t.debug_print("Bus being reset in " + to_string(i) + "seconds!\n");
            sleep_ms(1000);
        }
        gpio_put(vbus_reset_pin, true);
    }
    catch(...){
        t.debug_print("A failure occurred when trying to reset bus voltage!\n");
        error_count++;
    }
}

void pysquared::microcontroller_reset(){
    t.debug_print("Attempting to reset the microcontroller!\n");
    try{
        for(int i = 4; i > 0; i--){
            t.debug_print("Resetting the microcontroller in " + to_string(i) + "second(s)\n");
            sleep_ms(950);
        }
        AIRCR_Register = 0x5FA0004;
    }
    catch(...){
        t.debug_print("Failed to reset microcontroller!\n");
        error_count++;
    }
}

void pysquared::check_reboot(){
    try{
        t.debug_print("Checking for reboot...\n");
        uint32_t sec_since_boot = to_ms_since_boot(get_absolute_time())/1000;
        if(sec_since_boot>(60*60*24)){
            t.debug_print("Attempting to reboot!\n");
            bus_reset();
        }
        else{
            t.debug_print("No reboot yet.\n");
            return;
        }
    }
    catch(...){
        t.debug_print("ERROR while checking for reboot!\n");
        error_count++;
    }
}

void pysquared::all_faces_on(){
    try{
        t.debug_print("Attempting to turn all faces on!\n");
        led_driver.setPortState(0,true);
        led_driver.setPortState(1,true);
        led_driver.setPortState(2,true);
        led_driver.setPortState(3,true);
        led_driver.setPortState(4,true);
        faces_on_value=true;
        t.debug_print("all faces turned on!\n");
    }
    catch(...){
        t.debug_print("ERROR while turning all faces on!\n");
        error_count++;
    }
}

void pysquared::all_faces_off(){
    try{
        t.debug_print("Attempting to turn all faces off!\n");
        led_driver.setPortState(0,false);
        led_driver.setPortState(1,false);
        led_driver.setPortState(2,false);
        led_driver.setPortState(3,false);
        led_driver.setPortState(4,false);
        faces_on_value=false;
        t.debug_print("all faces turned off!\n");
    }
    catch(...){
        t.debug_print("ERROR while turning all faces off!\n");
        error_count++;
    }
}

void pysquared::five_volt_enable(){
    t.debug_print("Attempting to enable five volts...\n");
    try{
        gpio_put(five_volt_enable_pin, true);
        t.debug_print("five volts enabled!\n");
    }
    catch(...){
        t.debug_print("five volts could not be enabled!\n");
    }
}

void pysquared::five_volt_disable(){
    t.debug_print("Attempting to disable five volts...\n");
    try{
        gpio_put(five_volt_enable_pin, false);
        t.debug_print("five volts disabled!\n");
    }
    catch(...){
        t.debug_print("five volts could not be disabled!\n");
    }
}

void pysquared::flight_computer_off(){
    t.debug_print("Attempting to turn off flight computer...\n");
    try{
        gpio_put(fc_reset_pin, false);
        t.debug_print("flight computer off!\n");
    }
    catch(...){
        t.debug_print("flight computer could not be turned off!\n");
    }
}

void pysquared::flight_computer_on(){
    t.debug_print("Attempting to turn on flight computer...\n");
    try{
        gpio_put(fc_reset_pin, true);
        t.debug_print("flight computer on!\n");
    }
    catch(...){
        t.debug_print("flight computer could not be turned on!\n");
    }
}

void pysquared::camera_on(){
    try{
        t.debug_print("Attempting to turn the camera on!\n");
        led_driver.setPortState(5,true);
        t.debug_print("Camera turned on!\n");
        camera_on_value=true;
    }
    catch(...){
        t.debug_print("ERROR while turning the camera on!\n");
        error_count++;
    }
}

void pysquared::camera_off(){
    try{
        t.debug_print("Attempting to turn the camera off!\n");
        led_driver.setPortState(5,false);
        t.debug_print("Camera turned off!\n");
        camera_on_value=false;
    }
    catch(...){
        t.debug_print("ERROR while turning the camera off!\n");
        error_count++;
    }
}

void pysquared::heater_on(){
    try{
        t.debug_print("Turning Heater on...\n");
        gpio_put(relay_pin, true);
        pwm_set_enabled(heater_slice, true);
        pwm_set_wrap(heater_slice, 500); //500 as the wrap point will yield a 4us period (125MHz = 8ns => 8ns * 500 = 4us)
        pwm_set_chan_level(heater_slice, PWM_CHAN_A, 250); //This will yield a 50% duty cycle and a max average pwm voltage of 4.2V which is less than the 5V max they are rated for. (8.4V * 250/500 = 4.2V)
        t.debug_print("done!\n");
    }
    catch(...){
        t.debug_print("ERROR while turning on the heater!\n");
        pwm_set_chan_level(heater_slice, PWM_CHAN_A, 0);
        pwm_set_enabled(heater_slice,false);
        gpio_put(relay_pin, false);
        error_count++;
    }
}

void pysquared::heater_off(){
    try{
        t.debug_print("Turning Heater off...\n");
        pwm_set_chan_level(heater_slice, PWM_CHAN_A, 0);
        pwm_set_enabled(heater_slice,false);
        gpio_put(relay_pin, false);
        t.debug_print("done!\n");
    }
/*     catch(const std::exception &e){
        printf(e.what());
        error_count++;
    } */
    catch(...){
        t.debug_print("ERROR while turning off the heater!\n");
        gpio_put(relay_pin, false);
        error_count++;
    }
}

void pysquared::burn_on() {
    try {
        t.debug_print("Turning Burn Wire on...\n");
        
        // Enable the relay
        gpio_put(relay_pin, true);
        t.debug_print("Relay Pin On!");
        // Enable burn wire pin
        gpio_put(enable_burn_pin, true);
        t.debug_print("Burn Wire Pin On!");
        
        t.debug_print("Burn wire activated\n");
    }
    catch(...) {
        t.debug_print("ERROR while turning on the Burn Wire!\n");
        // Safety shutdown
        gpio_put(enable_burn_pin, false);
        gpio_put(relay_pin, false);
        error_count++;
    }
}

void pysquared::burn_off() {
    try {
        t.debug_print("Turning Burn Wire off...\n");
        
        // Disable burn wire pin
        gpio_put(enable_burn_pin, false);
        // Disable relay
        gpio_put(relay_pin, false);
        
        t.debug_print("Burn wire deactivated successfully\n");
    }
    catch(...) {
        t.debug_print("ERROR while turning off the Burn Wire!\n");
        // Force everything off in case of error
        gpio_put(enable_burn_pin, false);
        gpio_put(relay_pin, false);
        error_count++;
    }
}

bool pysquared::uart_send(const char *msg) {
    if (!msg || !uart0) {
        t.debug_print("Invalid UART or message pointer\n");
        return false;
    }
    
    t.debug_print("sending message: " + string(msg) + "\n");
    
    // Send ACK character first to indicate ready to send
    uart_write_blocking(uart0, (const uint8_t*)"<", 1);
    
    // Send the message
    uart_write_blocking(uart0, (const uint8_t*)msg, strlen(msg));
    
    // Send end marker
    uart_write_blocking(uart0, (const uint8_t*)">\n", 2);
    
    t.debug_print("Successful send!\n");
    return true;
}

void pysquared::uart_receive_handler() {
    t.debug_print("Checking for UART messages...\n");
    
    const int MAX_BYTES = 256; // Prevent infinite loop
    int counter = 0;
    uint8_t num;
    
    while (uart_is_readable(uart0) && counter < MAX_BYTES) {
        // Check for UART errors
        if (uart_is_readable_within_us(uart0, 1000)) {
            num = uart_getc(uart0);
            t.debug_print("Received: " + to_string(num) + "\n");
            
            // More robust command parsing
            if (isdigit(num)) {
                int cmd = num - '0';
                if (cmd >= 0 && cmd <= 11) {
                    exec_uart_command(cmd);
                } else {
                    t.debug_print("Invalid command number\n");
                }
            }
            counter++;
        } else {
            t.debug_print("UART read timeout\n");
            break;
        }
    }
    
    if (counter >= MAX_BYTES) {
        t.debug_print("Max receive buffer size reached\n");
    }
}

void pysquared::exec_uart_command(char commanded){
    // First send an ACK that we received the command
    uart_write_blocking(uart0, (const uint8_t*)"A", 1);

    string message;
    const char *msg;
    t.debug_print("Command #" + to_string(commanded) + " will be executed!\n");

    sleep_ms(10);
    uart_write_blocking(uart0, (const uint8_t*)"A", 1);
    switch (commanded)
    {
    case 1:
        t.debug_print("Executing command to send temperatures!\n");
        message = to_string(thermocouple_temp()) + "," + to_string(board_temp());
        msg=message.c_str();
        uart_send(msg);
        break;
    case 2:
        t.debug_print("Executing command to send Power Metrics!\n");
        message = to_string(battery_voltage()) + "," + to_string(draw_current()) + "," + to_string(charge_voltage()) + "," + to_string(charge_current()) + "," + to_string(is_charging());
        msg=message.c_str();
        uart_send(msg);
        break;
    case 3:
        t.debug_print("Executing command to send Error Metrics!\n");
        message = to_string(error_count) + "," + to_string(trust_memory);
        msg=message.c_str();
        uart_send(msg);
        break;
    case 4:
        t.debug_print("Executing command to toggle faces!\n"); //battery board usually makes this decision, but FC can override if important
        if(faces_on_value){
            all_faces_off();
        }
        else{
            all_faces_on();
        }
        message = "face value: " + to_string(faces_on_value);
        msg=message.c_str();
        uart_send(msg);
        break;
    case 5:
        t.debug_print("command received to reset entire power bus!\n");
        message="Consider it done...";
        msg=message.c_str();
        uart_send(msg);
        bus_reset();
        break;
    case 6:
        t.debug_print("command received to toggle camera power!\n");
        if(camera_on_value){
            camera_off();
        }
        else{
            camera_on();
        }
        message="camera value: " + to_string(camera_on_value);
        msg=message.c_str();
        uart_send(msg);
        break;
    case 7:
        t.debug_print("command received to start transmitting through auxillary radio!\n");
        //do some logic to trigger radio use!
        message = "will start using my own radio!";
        msg = message.c_str();
        uart_send(msg);
        break;
    case 8:
        t.debug_print("command received to reset flight controller!\n");
        message = "prepare to get power cycled... mwahahaha!";
        msg = message.c_str();
        uart_send(msg);
        sleep_ms(1000);
        flight_computer_off();
        sleep_ms(100);
        flight_computer_on();
        break;
    case 9:
        t.debug_print("command received to finish burn sequence!\n");
        burned = true;
        message = "duly noted!";
        msg=message.c_str();
        uart_send(msg);
        break;
    case 11:
        t.debug_print("command received to reset microcontroller!\nplease dont hurt me...\n");
        message = "see you soon!";
        msg = message.c_str();
        uart_send(msg);
        microcontroller_reset();
        break;
    case 208:
        t.debug_print("command error... attempting to resolve!\n");
        uart_get_hw(uart0)->rsr = 1;
        message = "error!";
        msg = message.c_str();
        uart_send(msg);
        break;
    default:
        t.debug_print("an invalid command was received!\n");
        break;
    }
}

uint8_t pysquared::power_mode() {return pwr_mode;}

float pysquared::thermocouple_temp(){
    try{
        float val=static_cast< float >(adc.readSingleEnded(1))*2.048/2048;
        printf("ADC Voltage measured: %.3f\n", val);
        return ((val-1.25)/0.004);
    }
    catch(...){
        t.debug_print("ERROR while getting thermocouple temperature!\n");
        error_count++;
        return 0;
    }
}
float pysquared::board_temp() {
    try{
        float temp;
        for(int i = 0; i<50; i++){
            temp+=internal_temp.readTemperature();
        }
        return temp/50;
    }
    catch(...){
        t.debug_print("ERROR while getting internal temperature!\n");
        error_count++;
        return 0;
    }
}
float pysquared::battery_voltage(){
    try{
        float temp;
        for(int i = 0; i<50; i++){
            temp+=battery_power.readBusVoltage();
        }
        return temp/50;
    }
    catch(...){
        t.debug_print("ERROR while getting battery voltage!\n");
        error_count++;
        return 0;
    }
}
float pysquared::draw_current(){
    try{
        float temp;
        for(int i = 0; i<50; i++){
            temp+=battery_power.readCurrent();
        }
        return temp/50;
    }
    catch(...){
        t.debug_print("ERROR while getting draw current!\n");
        error_count++;
        return 0;
    }
}
float pysquared::charge_voltage(){
    try{
        float temp;
        for(int i = 0; i<50; i++){
            temp+=solar_power.readBusVoltage();
        }
        return temp/50;
    }
    catch(...){
        t.debug_print("ERROR while getting charge voltage!\n");
        error_count++;
        return 0;
    }
}
float pysquared::charge_current(){
    try{
        float temp;
        for(int i = 0; i<50; i++){
            temp+=solar_power.readCurrent();
        }
    return temp/50;
    }
    catch(...){
        t.debug_print("ERROR while getting charge current!\n");
        error_count++;
        return 0;
    }
}

bool pysquared::is_charging(){
    try{
        return gpio_get(is_charge_pin);
    }
    catch(...){
        t.debug_print("Failure to read charge logic pin!\n");
        error_count++;
        return false;
    }
}

bool pysquared::charging(bool state){
    try{
        charge_status=state;
        return true;
    }
    catch(...){
        t.debug_print("ERROR setting charge status!\n");
        error_count++;
        return false;
    }
}

int pysquared::num_error(){return error_count;}




// CAN Bus Stuff