# Integration test: real accelero.c/ultrasonic.c drivers (copied verbatim
# from firmware/sensor_pico) plus the 3x VL53L1X ToF sensors, on the real
# sensor_pico pin map with ToF XSHUT moved to GP18/19/20.
#
# Added to the top-level 3TOF/CMakeLists.txt via:
#   add_subdirectory(integration_test)

add_executable(integration_test
    integration_test.c
    accelero.c
    ultrasonic.c
)

target_include_directories(integration_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(integration_test PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_timer
    VL53L1X_pico_api
)

# Pico W needs cyw43_arch for the onboard LED (see integration_test.c's
# led_set() comment for why GP25 can't be driven directly on Pico W).
if(${PICO_BOARD} STREQUAL pico_w)
    target_link_libraries(integration_test PUBLIC pico_cyw43_arch_none)
endif()

pico_add_extra_outputs(integration_test)
pico_enable_stdio_usb(integration_test 1)
pico_enable_stdio_uart(integration_test 0)
