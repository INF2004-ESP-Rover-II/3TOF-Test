/*
 * VL53L1X + Raspberry Pi Pico / Pico W
 *
 * SDA = GP4
 * SCL = GP5
 * I2C = i2c0
 * Address = 0x29
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#ifdef PICO_W_BOARD
#include "pico/cyw43_arch.h"
#endif

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"

#define I2C_DEV_ADDR 0x29

// Maximum amount of time to wait for a measurement
#define DATA_TIMEOUT_MS 1000


int main()
{
    VL53L1X_Status_t status = 0;
    VL53L1X_Result_t results;

    stdio_init_all();

    // Give USB serial time to connect
    sleep_ms(2000);

    printf("\n");
    printf("==============================\n");
    printf("VL53L1X Distance Sensor\n");
    printf("==============================\n");


    // -----------------------------------------
    // Initialize Pico / Pico W LED
    // -----------------------------------------

#ifdef PICO_W_BOARD

    if (cyw43_arch_init())
    {
        printf("Failed to initialize Pico W.\n");
        return 1;
    }

#else

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

#endif


    // -----------------------------------------
    // Blink LED to show program started
    // -----------------------------------------

    for (int i = 0; i < 4; i++)
    {

#ifdef PICO_W_BOARD

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

#else

        gpio_put(PICO_DEFAULT_LED_PIN, 1);

#endif

        sleep_ms(150);


#ifdef PICO_W_BOARD

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

#else

        gpio_put(PICO_DEFAULT_LED_PIN, 0);

#endif

        sleep_ms(150);
    }


    // -----------------------------------------
    // Initialize I2C
    //
    // Library uses:
    // I2C0
    // SDA = GP4
    // SCL = GP5
    // -----------------------------------------

    printf("Initializing I2C...\n");

    if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0)
    {
        printf("ERROR: Failed to initialize I2C.\n");
        return 1;
    }

    printf("I2C initialized.\n");


    // -----------------------------------------
    // Wait for VL53L1X to boot
    // -----------------------------------------

    printf("Waiting for sensor to boot...\n");

    uint8_t sensorState = 0;
    int bootTimeout = 0;

    do
    {
        status = VL53L1X_BootState(
            I2C_DEV_ADDR,
            &sensorState
        );

        if (status != 0)
        {
            printf("BootState error: %d\n", status);
        }

        VL53L1X_WaitMs(I2C_DEV_ADDR, 2);

        bootTimeout += 2;

        if (bootTimeout >= 2000)
        {
            printf("ERROR: Sensor boot timeout.\n");
            return 1;
        }

    } while (sensorState == 0);

    printf("Sensor booted successfully.\n");


    // -----------------------------------------
    // Initialize sensor
    // -----------------------------------------

    status = VL53L1X_SensorInit(I2C_DEV_ADDR);

    printf("SensorInit: %d\n", status);

    if (status != 0)
    {
        printf("ERROR: Sensor initialization failed.\n");
        return 1;
    }


    // -----------------------------------------
    // Distance mode
    //
    // 1 = Short
    // 2 = Long
    // -----------------------------------------

    status = VL53L1X_SetDistanceMode(
        I2C_DEV_ADDR,
        1
    );

    printf("SetDistanceMode: %d\n", status);

    if (status != 0)
    {
        printf("ERROR setting distance mode.\n");
        return 1;
    }


    // -----------------------------------------
    // Timing budget
    // -----------------------------------------

    status = VL53L1X_SetTimingBudgetInMs(
        I2C_DEV_ADDR,
        100
    );

    printf("SetTimingBudget: %d\n", status);

    if (status != 0)
    {
        printf("ERROR setting timing budget.\n");
        return 1;
    }


    // -----------------------------------------
    // Inter-measurement period
    //
    // Keep this greater than timing budget.
    // Timing budget = 100 ms
    // Inter measurement = 200 ms
    // -----------------------------------------

    status = VL53L1X_SetInterMeasurementInMs(
        I2C_DEV_ADDR,
        200
    );

    printf("SetInterMeasurement: %d\n", status);

    if (status != 0)
    {
        printf("ERROR setting inter-measurement period.\n");
        return 1;
    }


    // -----------------------------------------
    // Start ranging
    // -----------------------------------------

    status = VL53L1X_StartRanging(I2C_DEV_ADDR);

    printf("StartRanging: %d\n", status);

    if (status != 0)
    {
        printf("ERROR starting ranging.\n");
        return 1;
    }

    printf("\n");
    printf("==============================\n");
    printf("Starting measurements...\n");
    printf("==============================\n\n");


    // =========================================
    // MAIN MEASUREMENT LOOP
    // =========================================

    while (true)
{
    uint8_t dataReady = 0;
    int timeout = 0;

    // Start one measurement
    status = VL53L1X_StartRanging(I2C_DEV_ADDR);

    if (status != 0)
    {
        printf("StartRanging ERROR: %d\n", status);
        sleep_ms(100);
        continue;
    }

    // Wait for measurement
    while (dataReady == 0 && timeout < 1000)
    {
        status = VL53L1X_CheckForDataReady(
            I2C_DEV_ADDR,
            &dataReady
        );

        if (status != 0)
        {
            printf("CheckForDataReady ERROR: %d\n", status);
            break;
        }

        sleep_ms(5);
        timeout += 5;
    }

    if (status != 0)
    {
        VL53L1X_StopRanging(I2C_DEV_ADDR);
        sleep_ms(100);
        continue;
    }

    if (dataReady == 0)
    {
        printf("TIMEOUT waiting for measurement\n");

        VL53L1X_StopRanging(I2C_DEV_ADDR);

        sleep_ms(100);
        continue;
    }

    // Read result
    status = VL53L1X_GetResult(
        I2C_DEV_ADDR,
        &results
    );

    if (status != 0)
    {
        printf("GetResult ERROR: %d\n", status);

        VL53L1X_StopRanging(I2C_DEV_ADDR);

        sleep_ms(100);
        continue;
    }

    printf(
        "Status=%d | Distance=%d mm | "
        "Ambient=%d | Signal=%d | SPADs=%d\n",
        results.status,
        results.distance,
        results.ambient,
        results.sigPerSPAD,
        results.numSPADs
    );

    // Clear measurement
    status = VL53L1X_ClearInterrupt(I2C_DEV_ADDR);

    if (status != 0)
    {
        printf("ClearInterrupt ERROR: %d\n", status);
    }

    // Stop before next measurement
    status = VL53L1X_StopRanging(I2C_DEV_ADDR);

    if (status != 0)
    {
        printf("StopRanging ERROR: %d\n", status);
    }

    // Wait before next measurement
    sleep_ms(100);
}

    return 0;
}