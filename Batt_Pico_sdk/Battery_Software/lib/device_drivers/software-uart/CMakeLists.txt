# Create the library first
add_library(software_uart STATIC    # Add STATIC here to explicitly create a static library
    ${CMAKE_CURRENT_LIST_DIR}/software-uart.cpp    # Add your source files
    ${CMAKE_CURRENT_LIST_DIR}/software-uart.h     
)

# Generate the PIO header for this target
pico_generate_pio_header(software_uart
    ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio
)

# Add include directories
target_include_directories(software_uart PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}    # For generated files
    ${CMAKE_CURRENT_LIST_DIR}      # For your header files
)

# Link required libraries
target_link_libraries(software_uart PUBLIC
    pico_stdlib
    hardware_pio
)