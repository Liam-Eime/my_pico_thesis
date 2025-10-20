cmake_minimum_required(VERSION 3.13)

# Prefer environment-provided SDK path; fall back to local helper
if(DEFINED PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
else()
    # Uses the copy checked into this repo; can auto-fetch if configured there
    include(pico_sdk_import.cmake)
endif()

project(my_pico_thesis_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# This initializes the pico sdk (must be after include)
pico_sdk_init()

# Bring in the external SD/FatFS library
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/no_os_fatfs/src)

# Add your executable and its sources
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/hw_config.c
    src/adc_sampler.cpp
)

# Public include dir for shared headers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Link with Pico standard library and any hardware libs you need
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_spi
    hardware_dma
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    # etc, add more libs as needed
)

# Enable stdio over USB (or UART if you prefer)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Add UF2, map, and other extra outputs
pico_add_extra_outputs(${PROJECT_NAME})

# Export compile commands to improve IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
