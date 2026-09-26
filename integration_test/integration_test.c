/*
 * sensor_pico + 3x ToF integration test
 *
 * Proves three things before this gets merged into the real
 * firmware/sensor_pico/src/main.c:
 *
 *   1. The 3x VL53L1X can share I2C0 (GP4/GP5) with the LSM303DLHC IMU
 *      without either device losing readings.
 *   2. Moving ToF XSHUT off read_3_tofs.c's original GP6/GP7/GP8 onto
 *      GP18/GP19/GP20 does not break the left ultrasonic sensor, which
 *      already owns GP6/GP7.
 *   3. All four ultrasonic sensors, the IMU, and all three ToF sensors can
 *      be read in the same loop tick without one starving another.
 *
 * accelero.c/h and ultrasonic.c/h here are unmodified copies of the actual
 * firmware/sensor_pico driver files, not rewrites, so a pass here means the
 * real drivers work in this configuration.
 *
 * One real gotcha this file works around: accel_init() calls i2c_init() at
 * 100 kHz internally (see accelero.c). VL53L1X wants the bus at 400 kHz.
 * i2c_init() reconfigures the whole peripheral, so whichever call runs last
 * wins for every device on the bus. This file calls accel_init()/mag_init()
 * first, then re-inits I2C0 at 400 kHz before bringing up the ToF sensors.
 * The LSM303DLHC is fine at 400 kHz (Fast Mode), so nothing downstream
 * breaks, but this ordering matters. Get it backwards and the ToF sensors
 * either fail to init or return garbage.
 *
 * This test does NOT send UART packets to the Robo Pico or touch MQTT.
 * It only proves the sensor_pico side of the wiring works. UART packet
 * integration is a separate step once this passes.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#ifdef PICO_W_BOARD
#include "pico/cyw43_arch.h"
#endif

#include "sensor_pins.h"
#include "accelero.h"
#include "ultrasonic.h"
#include "VL53L1X_api.h"
#include "VL53L1X_types.h"

// ============================================================
// Status LED
// ============================================================
// The real sensor_pico main.c drives LED_PIN (GP25) directly with
// gpio_init(). That's correct on a plain Pico, but GP25 on a Pico W is
// wired internally to the CYW43439 wireless chip's SPI chip-select, not a
// free GPIO. read_3_tofs.c already does this the right way for Pico W
// (cyw43_arch_gpio_put), so this test follows that pattern instead of
// copying the direct-GPIO version. Worth fixing in the real firmware too
// if the board in hand is a Pico W.

static void led_set(bool on) {
#ifdef PICO_W_BOARD
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#else
    gpio_put(LED_PIN, on);
#endif
}

static void blink_error(void) {
    for (int i = 0; i < 3; i++) {
        led_set(true);
        sleep_ms(100);
        led_set(false);
        sleep_ms(100);
    }
}

static void blink_ok(void) {
    led_set(true);
    sleep_ms(50);
    led_set(false);
}

// ============================================================
// I2C bus scan (diagnostic only)
// ============================================================
// Confirms the bus actually shows the addresses we expect once everything
// is brought up: 0x19 (accel), 0x1E (mag), 0x30/0x31/0x32 (ToF LEFT/FRONT/
// RIGHT). Anything else on the bus, or a missing expected address, points
// at a wiring problem rather than a code problem.

static void i2c_scan(void) {
    printf("\n[I2C] Scanning i2c0 ...\n");
    int found = 0;

    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (ret >= 0) {
            const char *label = "";
            if (addr == 0x19) label = " (IMU accel)";
            else if (addr == 0x1E) label = " (IMU mag)";
            else if (addr == TOF_LEFT_ADDR) label = " (ToF LEFT)";
            else if (addr == TOF_FRONT_ADDR) label = " (ToF FRONT)";
            else if (addr == TOF_RIGHT_ADDR) label = " (ToF RIGHT)";
            printf("[I2C]   found 0x%02X%s\n", addr, label);
            found++;
        }
    }

    printf("[I2C] Scan complete, %d device(s) found\n\n", found);
}

// ============================================================
// ToF sensor bring-up
// ============================================================

typedef struct {
    uint8_t address;
    uint8_t xshut_pin;
    const char *name;
} tof_sensor_t;

static tof_sensor_t tof_sensors[3] = {
    {TOF_LEFT_ADDR,  TOF_LEFT_XSHUT,  "LEFT"},
    {TOF_FRONT_ADDR, TOF_FRONT_XSHUT, "FRONT"},
    {TOF_RIGHT_ADDR, TOF_RIGHT_XSHUT, "RIGHT"},
};

static bool tof_ok[3] = {false, false, false};

static void tof_init_xshut_pins(void) {
    for (int i = 0; i < 3; i++) {
        gpio_init(tof_sensors[i].xshut_pin);
        gpio_set_dir(tof_sensors[i].xshut_pin, GPIO_OUT);
        gpio_put(tof_sensors[i].xshut_pin, 0);  // all OFF initially
    }
    sleep_ms(100);
}

static bool tof_init_one(tof_sensor_t *s) {
    VL53L1X_Status_t status;

    printf("[ToF] Initializing %s (XSHUT=GP%d -> addr 0x%02X)...\n",
           s->name, s->xshut_pin, s->address);

    gpio_put(s->xshut_pin, 1);
    sleep_ms(100);  // boot time at default address 0x29

    status = VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDR, s->address << 1);
    if (status != 0) {
        printf("[ToF]   ERROR: SetI2CAddress failed (status=%d)\n", status);
        return false;
    }
    sleep_ms(10);

    status = VL53L1X_SensorInit(s->address);
    if (status != 0) {
        printf("[ToF]   ERROR: SensorInit failed (status=%d)\n", status);
        return false;
    }

    status = VL53L1X_SetDistanceMode(s->address, 2);  // 2 = long
    if (status != 0) {
        printf("[ToF]   ERROR: SetDistanceMode failed (status=%d)\n", status);
        return false;
    }

    status = VL53L1X_SetTimingBudgetInMs(s->address, 50);
    if (status != 0) {
        printf("[ToF]   ERROR: SetTimingBudget failed (status=%d)\n", status);
        return false;
    }

    status = VL53L1X_SetInterMeasurementInMs(s->address, 60);
    if (status != 0) {
        printf("[ToF]   ERROR: SetInterMeasurement failed (status=%d)\n", status);
        return false;
    }

    status = VL53L1X_StartRanging(s->address);
    if (status != 0) {
        printf("[ToF]   ERROR: StartRanging failed (status=%d)\n", status);
        return false;
    }

    printf("[ToF]   %s ready at 0x%02X\n", s->name, s->address);
    return true;
}

static bool tof_read_one(tof_sensor_t *s, uint16_t *distance_mm) {
    uint8_t data_ready = 0;
    if (VL53L1X_CheckForDataReady(s->address, &data_ready) != 0) return false;
    if (!data_ready) return false;
    if (VL53L1X_GetDistance(s->address, distance_mm) != 0) return false;
    VL53L1X_ClearInterrupt(s->address);
    return true;
}

// ============================================================
// Main
// ============================================================

int main(void) {
    stdio_init_all();
    sleep_ms(2000);  // let USB serial monitor attach

    printf("\n========================================\n");
    printf("  sensor_pico + 3x ToF integration test\n");
    printf("========================================\n\n");

#ifdef PICO_W_BOARD
    if (cyw43_arch_init()) {
        printf("[WARN] cyw43_arch_init failed, status LED disabled\n");
    }
#else
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
#endif

    // --------------------------------------------------------
    // IMU first (this also brings up I2C0 at 100 kHz internally)
    // --------------------------------------------------------
    printf("[IMU] Initializing accelerometer...\n");
    bool accel_available = (accel_init() == ACCEL_OK);
    printf("[IMU] Accelerometer: %s\n", accel_available ? "OK" : "FAILED");
    if (!accel_available) blink_error();

    printf("[IMU] Initializing magnetometer...\n");
    bool mag_available = (mag_init() == ACCEL_OK);
    printf("[IMU] Magnetometer: %s\n", mag_available ? "OK" : "FAILED");
    if (!mag_available) blink_error();

    // --------------------------------------------------------
    // Ultrasonic sensors (plain GPIO, no bus contention)
    // --------------------------------------------------------
    printf("\n[US] Initializing 4 ultrasonic sensors...\n");
    ultrasonic_init(ULTRA_FRONT_TRIG_PIN, ULTRA_FRONT_ECHO_PIN);
    ultrasonic_init(ULTRA_BACK_TRIG_PIN, ULTRA_BACK_ECHO_PIN);
    ultrasonic_init(ULTRA_LEFT_TRIG_PIN, ULTRA_LEFT_ECHO_PIN);
    ultrasonic_init(ULTRA_RIGHT_TRIG_PIN, ULTRA_RIGHT_ECHO_PIN);
    printf("[US] 4 sensors initialized (front/back/left/right)\n");

    // --------------------------------------------------------
    // Re-arm I2C0 at 400 kHz before touching the ToF sensors.
    // See the file header comment: accel_init() left the bus at 100 kHz.
    // --------------------------------------------------------
    printf("\n[I2C] Re-initializing i2c0 at 400 kHz for ToF...\n");
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    VL53L1X_I2C_Init(TOF_DEFAULT_ADDR, I2C_PORT);  // records i2c handle only

    // --------------------------------------------------------
    // ToF sensors, one at a time via XSHUT
    // --------------------------------------------------------
    printf("\n[ToF] Holding all 3 sensors in reset...\n");
    tof_init_xshut_pins();

    printf("[ToF] Bringing sensors up one at a time...\n");
    int tof_ok_count = 0;
    for (int i = 0; i < 3; i++) {
        tof_ok[i] = tof_init_one(&tof_sensors[i]);
        if (!tof_ok[i]) {
            printf("[ToF] %s did not come up, continuing with the rest\n",
                   tof_sensors[i].name);
            blink_error();
        } else {
            tof_ok_count++;
        }
    }
    printf("[ToF] %d of 3 sensors ready\n", tof_ok_count);

    // --------------------------------------------------------
    // Bus scan: confirms the final address layout on i2c0
    // --------------------------------------------------------
    i2c_scan();

    printf("========================================\n");
    printf("  Bring-up summary\n");
    printf("  IMU accel:   %s\n", accel_available ? "ONLINE" : "OFFLINE");
    printf("  IMU mag:     %s\n", mag_available ? "ONLINE" : "OFFLINE");
    printf("  Ultrasonic:  ONLINE (front/back/left/right)\n");
    printf("  ToF LEFT:    %s\n", tof_ok[0] ? "ONLINE" : "OFFLINE");
    printf("  ToF FRONT:   %s\n", tof_ok[1] ? "ONLINE" : "OFFLINE");
    printf("  ToF RIGHT:   %s\n", tof_ok[2] ? "ONLINE" : "OFFLINE");
    printf("========================================\n\n");

    blink_ok();

    // --------------------------------------------------------
    // Main loop: read everything, once per 100 ms, print one line.
    // Also tracks consecutive good reads per subsystem so a single lucky
    // read doesn't get mistaken for "it works" per the lecture's point
    // about testing rigorously rather than trusting a first pass.
    // --------------------------------------------------------
    uint32_t loop_count = 0;
    uint32_t imu_ok_streak = 0, us_ok_streak = 0, tof_ok_streak = 0;
    uint32_t last_summary_ms = 0;

    uint16_t tof_distance[3] = {0, 0, 0};

    printf("[SYSTEM] Entering main loop (100 ms tick)...\n\n");

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        loop_count++;

        // ---- IMU ----
        float heading_deg = 0, pitch_deg = 0;
        bool imu_tick_ok = false;
        if (accel_available && mag_available) {
            float ax, ay, az, mx, my, mz;
            if (read_accel(&ax, &ay, &az) == ACCEL_OK &&
                read_mag(&mx, &my, &mz) == ACCEL_OK) {
                compute_tilt_compensated_heading(ax, ay, az, mx, my, mz, &heading_deg);
                float roll_deg;
                compute_tilt(ax, ay, az, &roll_deg, &pitch_deg);
                imu_tick_ok = true;
            }
        }
        imu_ok_streak = imu_tick_ok ? imu_ok_streak + 1 : 0;

        // ---- Ultrasonic ----
        float us_front = -1, us_back = -1, us_left = -1, us_right = -1;
        bool us_tick_ok = true;
        us_tick_ok &= (ultrasonic_get_cm(ULTRA_FRONT_TRIG_PIN, ULTRA_FRONT_ECHO_PIN, &us_front) == ULTRASONIC_OK);
        sleep_ms(15);
        us_tick_ok &= (ultrasonic_get_cm(ULTRA_BACK_TRIG_PIN, ULTRA_BACK_ECHO_PIN, &us_back) == ULTRASONIC_OK);
        sleep_ms(15);
        us_tick_ok &= (ultrasonic_get_cm(ULTRA_LEFT_TRIG_PIN, ULTRA_LEFT_ECHO_PIN, &us_left) == ULTRASONIC_OK);
        sleep_ms(15);
        us_tick_ok &= (ultrasonic_get_cm(ULTRA_RIGHT_TRIG_PIN, ULTRA_RIGHT_ECHO_PIN, &us_right) == ULTRASONIC_OK);
        us_ok_streak = us_tick_ok ? us_ok_streak + 1 : 0;

        // ---- ToF ----
        bool tof_tick_ok = true;
        for (int i = 0; i < 3; i++) {
            if (!tof_ok[i]) { tof_tick_ok = false; continue; }
            if (!tof_read_one(&tof_sensors[i], &tof_distance[i])) {
                tof_tick_ok = false;  // stale reading kept, not overwritten
            }
        }
        tof_ok_streak = tof_tick_ok ? tof_ok_streak + 1 : 0;

        // ---- One-line status ----
        if (imu_tick_ok) {
            printf("[%6lu ms] IMU hdg=%6.1f pitch=%5.1f | ", now_ms, heading_deg, pitch_deg);
        } else {
            printf("[%6lu ms] IMU offline           | ", now_ms);
        }
        printf("US F=%3.0f B=%3.0f L=%3.0f R=%3.0f cm | ", us_front, us_back, us_left, us_right);
        printf("ToF L=%4u F=%4u R=%4u mm\n",
               tof_ok[0] ? tof_distance[0] : 0,
               tof_ok[1] ? tof_distance[1] : 0,
               tof_ok[2] ? tof_distance[2] : 0);

        // ---- Health summary every 5 s ----
        if (now_ms - last_summary_ms >= 5000) {
            last_summary_ms = now_ms;
            printf("\n[HEALTH] loop=%lu | IMU streak=%lu | US streak=%lu | ToF streak=%lu\n\n",
                   loop_count, imu_ok_streak, us_ok_streak, tof_ok_streak);
            blink_ok();
        }

        sleep_ms(100);
    }

    return 0;
}
