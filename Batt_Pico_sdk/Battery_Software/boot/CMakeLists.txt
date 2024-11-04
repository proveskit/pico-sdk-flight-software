if (TARGET tinyusb_device)
    add_executable(boot boot.cpp)
    
    # Generate PIO headers
    pico_generate_pio_header(boot ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
    pico_generate_pio_header(boot ${CMAKE_CURRENT_LIST_DIR}/neopixel.pio)
    
    # Link all required libraries
    target_link_libraries(boot PRIVATE
        pico_stdlib
        hardware_pio
        hardware_gpio
        hardware_i2c
        main
        neopixel
        tools
        pysquared
        functions
        big_data
        ina219
        pct2075
        pca9685
        tca9548a
        mcp9808
        veml7700
        drv2605
        ads1015
    )
    
    # Enable USB output, disable UART
    pico_enable_stdio_usb(boot 1)
    pico_enable_stdio_uart(boot 0)
    
    # Generate additional outputs (UF2, etc.)
    pico_add_extra_outputs(boot)
    
    # Add URL via pico_set_program_url
    example_auto_set_url(boot)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
