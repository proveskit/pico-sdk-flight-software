if (NOT PICO_NO_HARDWARE)
    add_subdirectory(lib)
    add_subdirectory(boot)
endif ()

target_include_directories(main PUBLIC lib)
target_include_directories(neopixel PUBLIC lib)
target_include_directories(tools PUBLIC lib)
target_include_directories(pysquared PUBLIC lib)
target_include_directories(ina219 PUBLIC lib)
target_include_directories(pct2075 PUBLIC lib)
target_include_directories(pca9685 PUBLIC lib)
target_include_directories(vl6180 PUBLIC lib)
target_include_directories(ads1015 PUBLIC lib)
target_include_directories(mcp25625 PUBLIC lib)
target_include_directories(functions PUBLIC lib)

pico_enable_stdio_usb(main 1)