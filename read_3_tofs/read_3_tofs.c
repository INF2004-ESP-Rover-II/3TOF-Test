/*
 * 3x VL53L1X + Raspberry Pi Pico W
 *
 * Shared I2C bus:
 *   SDA = GP4
 *   SCL = GP5
 *   I2C = i2c0
 *
 * XSHUT:
 *   Sensor 1 = GP6
 *   Sensor 2 = GP7
 *   Sensor 3 = GP8
 *
 * Assigned I2C addresses:
 *   Sensor 1 = 0x30
 *   Sensor 2 = 0x31
 *   Sensor 3 = 0x32
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef PICO_W_BOARD
#include "pico/cyw43_arch.h"
#endif

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"


// ============================================================
// I2C configuration
// ============================================================

#define I2C_PORT i2c0
#define SDA_PIN  4
#define SCL_PIN  5

#define DEFAULT_ADDR 0x29


// ============================================================
// Sensor configuration
// ============================================================

#define NUM_SENSORS 3

#define SENSOR_1_XSHUT 6
#define SENSOR_2_XSHUT 7
#define SENSOR_3_XSHUT 8

#define SENSOR_1_ADDR 0x30
#define SENSOR_2_ADDR 0x31
#define SENSOR_3_ADDR 0x32


// ============================================================
// Sensor structure
// ============================================================

typedef struct
{
    uint8_t address;
    uint8_t xshut_pin;
    const char *name;
} ToFSensor;


ToFSensor sensors[NUM_SENSORS] =
{
    {SENSOR_1_ADDR, SENSOR_1_XSHUT, "LEFT"},
    {SENSOR_2_ADDR, SENSOR_2_XSHUT, "FRONT"},
    {SENSOR_3_ADDR, SENSOR_3_XSHUT, "RIGHT"}
};


// ============================================================
// XSHUT helper functions
// ============================================================

void sensor_shutdown(uint8_t pin)
{
    gpio_put(pin, 0);
}


void sensor_enable(uint8_t pin)
{
    gpio_put(pin, 1);

    // Give sensor time to boot
    sleep_ms(100);
}


// ============================================================
// Initialize XSHUT pins
// ============================================================

void init_xshut_pins()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        gpio_init(sensors[i].xshut_pin);
        gpio_set_dir(sensors[i].xshut_pin, GPIO_OUT);

        // Keep all sensors OFF initially
        gpio_put(sensors[i].xshut_pin, 0);
    }

    sleep_ms(100);
}


// ============================================================
// Initialize I2C
// ============================================================

void init_i2c()
{
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // The VL53L1X ULD driver (VL53L1X_platform.c) keeps its own internal
    // i2c_inst_t* handle, which is ONLY ever set inside VL53L1X_I2C_Init().
    // Without calling it, that handle stays NULL and every I2C read/write
    // the driver performs silently fails, regardless of wiring. No sensor
    // is powered yet at this point (all XSHUT pins are still low), so the
    // sensor-ID check inside VL53L1X_I2C_Init() is expected to fail here -
    // we only care about its side effect of recording the i2c instance,
    // so the return value is intentionally ignored.
    VL53L1X_I2C_Init(DEFAULT_ADDR, I2C_PORT);
}


// ============================================================
// Initialize one VL53L1X
// ============================================================

bool init_sensor(ToFSensor *sensor)
{
    VL53L1X_Status_t status;

    printf("\nInitializing %s sensor...\n", sensor->name);

    // Sensor is currently OFF.
    // Turn it on. It will start at default address 0x29.
    sensor_enable(sensor->xshut_pin);

    printf("  Sensor booted at 0x%02X\n", DEFAULT_ADDR);


    // --------------------------------------------------------
    // Change I2C address
    // --------------------------------------------------------

    printf(
        "  Changing address 0x%02X -> 0x%02X...\n",
        DEFAULT_ADDR,
        sensor->address
    );

    printf("BEFORE SetI2CAddress\n");

status = VL53L1X_SetI2CAddress(
    DEFAULT_ADDR,
    sensor->address << 1
);

printf("AFTER SetI2CAddress, status = %d\n", status);

    if (status != 0)
    {
        printf(
            "  ERROR: Failed to change address. Status = %d\n",
            status
        );

        return false;
    }

    sleep_ms(10);

    printf("  Address changed successfully.\n");


    // --------------------------------------------------------
    // Sensor initialization
    // --------------------------------------------------------

    printf("  Running sensor initialization...\n");

    status = VL53L1X_SensorInit(sensor->address);

    if (status != 0)
    {
        printf(
            "  ERROR: SensorInit failed. Status = %d\n",
            status
        );

        return false;
    }


    // --------------------------------------------------------
    // Configure distance mode
    // --------------------------------------------------------

    // 1 = Short
    // 2 = Long

    status = VL53L1X_SetDistanceMode(
        sensor->address,
        2
    );

    if (status != 0)
    {
        printf(
            "  ERROR: SetDistanceMode failed. Status = %d\n",
            status
        );

        return false;
    }


    // --------------------------------------------------------
    // Timing budget
    // --------------------------------------------------------

    status = VL53L1X_SetTimingBudgetInMs(
        sensor->address,
        50
    );

    if (status != 0)
    {
        printf(
            "  ERROR: SetTimingBudget failed. Status = %d\n",
            status
        );

        return false;
    }


    // --------------------------------------------------------
    // Inter-measurement period
    // --------------------------------------------------------

    status = VL53L1X_SetInterMeasurementInMs(
        sensor->address,
        60
    );

    if (status != 0)
    {
        printf(
            "  ERROR: SetInterMeasurement failed. Status = %d\n",
            status
        );

        return false;
    }


    // --------------------------------------------------------
    // Start ranging
    // --------------------------------------------------------

    status = VL53L1X_StartRanging(sensor->address);

    if (status != 0)
    {
        printf(
            "  ERROR: StartRanging failed. Status = %d\n",
            status
        );

        return false;
    }

    printf(
        "  %s ready at address 0x%02X\n",
        sensor->name,
        sensor->address
    );

    return true;
}


// ============================================================
// Read one sensor
// ============================================================

bool read_sensor(ToFSensor *sensor, uint16_t *distance)
{
    VL53L1X_Status_t status;
    uint8_t data_ready = 0;


    // Check whether a new measurement is available
    status = VL53L1X_CheckForDataReady(
        sensor->address,
        &data_ready
    );


    if (status != 0)
    {
        return false;
    }


    if (!data_ready)
    {
        return false;
    }


    // Get distance
    status = VL53L1X_GetDistance(
        sensor->address,
        distance
    );

    if (status != 0)
    {
        return false;
    }


    // Clear interrupt so sensor can take next reading
    status = VL53L1X_ClearInterrupt(sensor->address);

    if (status != 0)
    {
        return false;
    }


    return true;
}


// ============================================================
// Main
// ============================================================

int main()
{
    stdio_init_all();

    // Allow USB serial monitor to connect
    sleep_ms(2000);


    printf("\n");
    printf("========================================\n");
    printf("  3x VL53L1X Distance Sensor System\n");
    printf("========================================\n");


    // --------------------------------------------------------
    // Initialize Pico W LED
    // --------------------------------------------------------

#ifdef PICO_W_BOARD

    if (cyw43_arch_init())
    {
        printf("WARNING: Failed to initialize Pico W LED\n");
    }
    else
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }

#endif


    // --------------------------------------------------------
    // Initialize I2C
    // --------------------------------------------------------

    printf("\nInitializing I2C...\n");

    init_i2c();

    printf("I2C initialized.\n");
    printf("SDA = GP%d\n", SDA_PIN);
    printf("SCL = GP%d\n", SCL_PIN);


    // --------------------------------------------------------
    // Initialize XSHUT
    // --------------------------------------------------------

    printf("\nShutting down all sensors...\n");

    init_xshut_pins();

    printf("All sensors shutdown.\n");


    // --------------------------------------------------------
    // Initialize sensors ONE AT A TIME
    // --------------------------------------------------------

    printf("\n");
    printf("========================================\n");
    printf("Initializing sensors\n");
    printf("========================================\n");


    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (!init_sensor(&sensors[i]))
        {
            printf(
                "\nFATAL ERROR: Failed to initialize %s sensor.\n",
                sensors[i].name
            );

            while (true)
            {
                sleep_ms(1000);
            }
        }
    }


    printf("\n");
    printf("========================================\n");
    printf("All sensors initialized!\n");
    printf("========================================\n");

    printf("LEFT  = 0x%02X\n", sensors[0].address);
    printf("FRONT = 0x%02X\n", sensors[1].address);
    printf("RIGHT = 0x%02X\n", sensors[2].address);

    printf("\nStarting measurements...\n\n");


    // ========================================================
    // Measurement loop
    // ========================================================

    uint16_t distances[NUM_SENSORS] = {0};


    while (true)
    {
        bool updated[NUM_SENSORS] =
        {
            false,
            false,
            false
        };


        // ----------------------------------------------------
        // Read all three sensors
        // ----------------------------------------------------

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            updated[i] = read_sensor(
                &sensors[i],
                &distances[i]
            );
        }


        // ----------------------------------------------------
        // Print readings
        // ----------------------------------------------------

        // Prints whichever sensors are actually in the sensors[] array
        // right now (NUM_SENSORS-driven), instead of a hardcoded 3-column
        // line - keeps working correctly while testing with fewer sensors.
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            printf("%s: %4u mm", sensors[i].name, distances[i]);
            if (i < NUM_SENSORS - 1)
            {
                printf(" | ");
            }
        }
        printf("\n");


        // Don't spam USB serial
        sleep_ms(50);
    }


    return 0;
}