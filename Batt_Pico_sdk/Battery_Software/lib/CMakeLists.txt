# Main libraries
add_library(main 
    main/main.h 
    main/main.cpp
)

add_library(neopixel 
    neopixel/neopixel.h 
    neopixel/neopixel.cpp
)

add_library(tools 
    tools/tools.h 
    tools/tools.cpp
)

add_library(pysquared 
    pysquared/pysquared.h 
    pysquared/pysquared.cpp
)

add_library(functions 
    functions/functions.h 
    functions/functions.cpp
)

# Device drivers
add_library(ina219 
    device_drivers/INA219.h 
    device_drivers/INA219.cpp
)

add_library(pct2075 
    device_drivers/PCT2075.h 
    device_drivers/PCT2075.cpp
)

add_library(pca9685 
    device_drivers/PCA9685.h 
    device_drivers/PCA9685.cpp
)

add_library(vl6180 
    device_drivers/VL6180.h 
    device_drivers/VL6180.cpp
)

add_library(ads1015 
    device_drivers/ADS1015.h 
    device_drivers/ADS1015.cpp
)

add_library(mcp25625 
    device_drivers/MCP25625.h 
    device_drivers/MCP25625.cpp
)

# Library dependencies
target_link_libraries(main PRIVATE 
    pico_stdlib 
    neopixel 
    hardware_pio 
    hardware_i2c 
    hardware_pwm 
    hardware_spi 
    hardware_uart 
    hardware_irq 
    hardware_watchdog 
    hardware_flash 
    hardware_sync 
    tools 
    pysquared 
    functions
)

target_link_libraries(neopixel PRIVATE 
    hardware_pio 
    pico_stdlib
)

target_link_libraries(tools PRIVATE
    pico_stdlib
    hardware_pio
    hardware_i2c
    hardware_pwm
    hardware_spi
    hardware_timer
    hardware_watchdog
    hardware_flash
    hardware_clocks
)

target_link_libraries(pysquared PRIVATE 
    pico_stdlib 
    hardware_flash 
    hardware_sync 
    hardware_pio 
    hardware_pwm 
    hardware_gpio 
    hardware_i2c 
    hardware_spi 
    hardware_uart 
    hardware_irq 
    neopixel 
    tools 
    functions 
    ina219 
    pct2075 
    pca9685 
    vl6180 
    ads1015 
    mcp25625
)

# Device driver dependencies
target_link_libraries(ina219 PRIVATE 
    pico_stdlib 
    hardware_i2c
)

target_link_libraries(pct2075 PRIVATE 
    pico_stdlib 
    hardware_i2c
)

target_link_libraries(pca9685 PRIVATE 
    pico_stdlib 
    hardware_i2c
)

target_link_libraries(vl6180 PRIVATE 
    pico_stdlib 
    hardware_i2c
)

target_link_libraries(ads1015 PRIVATE 
    pico_stdlib 
    hardware_i2c
)

target_link_libraries(mcp25625 PRIVATE
    pico_stdlib 
    hardware_spi 
    hardware_gpio
)

target_link_libraries(functions PRIVATE 
    pico_stdlib 
    hardware_flash 
    hardware_sync 
    hardware_pio 
    hardware_gpio 
    hardware_pwm 
    hardware_i2c 
    hardware_spi 
    hardware_uart 
    hardware_irq 
    hardware_watchdog 
    neopixel 
    tools 
    pysquared 
    vl6180 
    mcp25625
)