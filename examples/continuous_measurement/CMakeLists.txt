# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

add_executable(continuous continuous.c)

# Link to pico libraries as well as the VL53L1X_pico_api
target_link_libraries(continuous PUBLIC pico_stdlib hardware_i2c VL53L1X_pico_api)

# Add needed library if Pico W
if(${PICO_BOARD} STREQUAL pico_w)
    target_link_libraries(continuous PUBLIC pico_cyw43_arch_none)
endif()

# Generate executable, enabling STDIO over USB
pico_add_extra_outputs(continuous)
pico_enable_stdio_usb(continuous 1)