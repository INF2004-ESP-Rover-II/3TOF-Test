#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "VL53L1X_api.h"
#include "VL53L1X_platform.h"

// ======================================================
// VL53L1X ADDRESSES
// ======================================================

#define DEFAULT_ADDR 0x29

#define FRONT_ADDR 0x30
#define LEFT_ADDR 0x31
#define RIGHT_ADDR 0x32

// ======================================================
// XSHUT PINS
// ======================================================

#define FRONT_XSHUT 6
#define LEFT_XSHUT 7
#define RIGHT_XSHUT 8

// ======================================================
// SIMULATED ODOMETRY BUTTON
// ======================================================

#define MOVE_BUTTON 20

// Every button press = 100 mm forward
#define MOVE_DISTANCE 100

// ======================================================
// SIMULATED ROVER POSITION
// ======================================================

// We start at (0, 0)
//
// For this test:
// X = left/right
// Y = forward/backward
//
// Rover always faces +Y.

int32_t rover_x = 0;
int32_t rover_y = 0;

// ======================================================
// INITIALISE ONE SENSOR
// ======================================================

int init_sensor(
    uint xshut_pin,
    uint8_t new_addr,
    const char *name)
{
    uint8_t boot_state = 0;

    printf("Starting %s...\n", name);

    // Wake sensor
    gpio_put(xshut_pin, 1);

    sleep_ms(100);

    // --------------------------------------------------
    // Wait for sensor boot
    // --------------------------------------------------

    for (int i = 0; i < 100; i++)
    {
        printf("%s boot attempt %d...\n", name, i);

        int status = VL53L1X_BootState(
            DEFAULT_ADDR,
            &boot_state);

        printf(
            "%s BootState returned %d, boot_state=%u\n",
            name,
            status,
            boot_state);

        if (status != 0)
        {
            printf(
                "ERROR: %s I2C communication failed during boot\n",
                name);
            return -1;
        }

        if (boot_state)
            break;

        sleep_ms(10);
    }

    if (!boot_state)
    {
        printf(
            "ERROR: %s failed to boot\n",
            name);

        return -1;
    }

    // --------------------------------------------------
    // Give sensor unique I2C address
    // --------------------------------------------------

    if (VL53L1X_SetI2CAddress(
            DEFAULT_ADDR,
            new_addr << 1) != 0)
    {
        printf(
            "ERROR: %s address change failed\n",
            name);

        return -1;
    }

    sleep_ms(10);

    // --------------------------------------------------
    // Sensor initialization
    // --------------------------------------------------

    if (VL53L1X_SensorInit(new_addr) != 0)
    {
        printf(
            "ERROR: %s SensorInit failed\n",
            name);

        return -1;
    }

    // Long-distance mode
    VL53L1X_SetDistanceMode(
        new_addr,
        2);

    // 50 ms timing budget
    VL53L1X_SetTimingBudgetInMs(
        new_addr,
        50);

    // Start ranging
    VL53L1X_StartRanging(
        new_addr);

    printf(
        "%s ready at 0x%02X\n",
        name,
        new_addr);

    return 0;
}

// ======================================================
// READ ONE SENSOR
// ======================================================

int read_sensor(
    uint8_t addr,
    uint16_t *distance)
{
    uint8_t ready = 0;
    uint8_t status = 255;

    // --------------------------------------------------
    // Check if measurement ready
    // --------------------------------------------------

    if (VL53L1X_CheckForDataReady(
            addr,
            &ready) != 0)
    {
        return -1;
    }

    if (!ready)
        return 0;

    // --------------------------------------------------
    // Get distance
    // --------------------------------------------------

    if (VL53L1X_GetDistance(
            addr,
            distance) != 0)
    {
        return -1;
    }

    // --------------------------------------------------
    // Get measurement status
    // --------------------------------------------------

    if (VL53L1X_GetRangeStatus(
            addr,
            &status) != 0)
    {
        return -1;
    }

    VL53L1X_ClearInterrupt(addr);

    // Status 0 = valid measurement
    if (status != 0)
        return -1;

    return 1;
}

// ======================================================
// MAIN
// ======================================================

int main(void)
{
    stdio_init_all();

    sleep_ms(2000);

    printf("\n");
    printf("=================================\n");
    printf(" 3-ToF Mapping + Fake Odometry\n");
    printf("=================================\n\n");

    // ==================================================
    // I2C SETUP
    // ==================================================

    i2c_init(
        i2c_default,
        VL53L1X_I2C_BAUDRATE);

    gpio_set_function(
        PICO_DEFAULT_I2C_SDA_PIN,
        GPIO_FUNC_I2C);

    gpio_set_function(
        PICO_DEFAULT_I2C_SCL_PIN,
        GPIO_FUNC_I2C);

    gpio_pull_up(
        PICO_DEFAULT_I2C_SDA_PIN);

    gpio_pull_up(
        PICO_DEFAULT_I2C_SCL_PIN);

    // ==================================================
    // XSHUT SETUP
    // ==================================================

    gpio_init(FRONT_XSHUT);
    gpio_init(LEFT_XSHUT);
    gpio_init(RIGHT_XSHUT);

    gpio_set_dir(
        FRONT_XSHUT,
        GPIO_OUT);

    gpio_set_dir(
        LEFT_XSHUT,
        GPIO_OUT);

    gpio_set_dir(
        RIGHT_XSHUT,
        GPIO_OUT);

    // Turn all sensors off
    gpio_put(FRONT_XSHUT, 0);
    gpio_put(LEFT_XSHUT, 0);
    gpio_put(RIGHT_XSHUT, 0);

    sleep_ms(100);

    // ==================================================
    // BUTTON SETUP
    // ==================================================

    gpio_init(MOVE_BUTTON);

    gpio_set_dir(
        MOVE_BUTTON,
        GPIO_IN);

    gpio_pull_up(
        MOVE_BUTTON);

    // ==================================================
    // DRIVER INITIALIZATION
    // ==================================================

    gpio_put(
        FRONT_XSHUT,
        1);

    sleep_ms(100);

    if (VL53L1X_I2C_Init(
            DEFAULT_ADDR,
            i2c_default) != 0)
    {
        printf(
            "ERROR: I2C initialization failed\n");

        while (1)
            sleep_ms(1000);
    }

    gpio_put(
        FRONT_XSHUT,
        0);

    sleep_ms(100);

    // ==================================================
    // INITIALISE THREE SENSORS
    // ==================================================

    if (init_sensor(
            FRONT_XSHUT,
            FRONT_ADDR,
            "FRONT") != 0)
    {
        while (1)
            sleep_ms(1000);
    }

    if (init_sensor(
            LEFT_XSHUT,
            LEFT_ADDR,
            "LEFT") != 0)
    {
        while (1)
            sleep_ms(1000);
    }

    if (init_sensor(
            RIGHT_XSHUT,
            RIGHT_ADDR,
            "RIGHT") != 0)
    {
        while (1)
            sleep_ms(1000);
    }

    printf("\nAll sensors ready.\n\n");

    printf(
        "Start position: X=%ld Y=%ld\n",
        (long)rover_x,
        (long)rover_y);

    printf(
        "Press GP20 = simulate 100 mm forward\n\n");

    // ==================================================
    // SENSOR VALUES
    // ==================================================

    uint16_t front = 0;
    uint16_t left = 0;
    uint16_t right = 0;

    bool have_front = false;
    bool have_left = false;
    bool have_right = false;

    // ==================================================
    // BUTTON STATE
    // ==================================================

    bool previous_button = true;

    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
        // ==================================================
        // CHECK GP20 BUTTON
        // ==================================================

        bool current_button =
            gpio_get(MOVE_BUTTON);

        // Button uses pull-up:
        //
        // Not pressed = HIGH
        // Pressed     = LOW
        //
        // Detect HIGH -> LOW transition

        if (
            previous_button == true &&
            current_button == false)
        {
            // ------------------------------------------
            // SIMULATE 100 mm forward movement
            // ------------------------------------------

            rover_y += MOVE_DISTANCE;

            printf(
                "MOVE: X=%ld Y=%ld\n",
                (long)rover_x,
                (long)rover_y);

            // Simple debounce
            sleep_ms(200);
        }

        previous_button =
            current_button;

        // ==================================================
        // READ SENSORS
        // ==================================================

        int f = read_sensor(
            FRONT_ADDR,
            &front);

        int l = read_sensor(
            LEFT_ADDR,
            &left);

        int r = read_sensor(
            RIGHT_ADDR,
            &right);

        if (f == 1)
            have_front = true;

        if (l == 1)
            have_left = true;

        if (r == 1)
            have_right = true;

        // ==================================================
        // SEND DATA TO PYTHON
        // ==================================================

        if (
            have_front &&
            have_left &&
            have_right)
        {
            printf(
                "MAP,%ld,%ld,%u,%u,%u\n",
                (long)rover_x,
                (long)rover_y,
                front,
                left,
                right);

            have_front = false;
            have_left = false;
            have_right = false;
        }

        sleep_ms(5);
    }
}