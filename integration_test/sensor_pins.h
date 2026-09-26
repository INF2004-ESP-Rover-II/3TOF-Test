#ifndef SENSOR_PINS_H
#define SENSOR_PINS_H

// Pin map for the sensor_pico + 3x ToF integration test.
//
// IMU and ultrasonic pins mirror firmware/sensor_pico/include/sensor_config.h
// in the Senior Project repo exactly, so this test proves the real pin
// layout, not a stand-in one.
//
// ToF XSHUT pins are moved from read_3_tofs.c's original GP6/GP7/GP8 to
// GP18/GP19/GP20. GP6/GP7 are ULTRA_LEFT_TRIG_PIN/ULTRA_LEFT_ECHO_PIN on the
// real board, so the original XSHUT wiring would have fought the left
// ultrasonic sensor for those pins. GP18/19/20 are unused anywhere else in
// sensor_pico's pin map.

// ====================== IMU (LSM303DLHC) ======================
#define I2C_PORT        i2c0
#define SDA_PIN         4   // GP4
#define SCL_PIN         5   // GP5

// ====================== ULTRASONIC (HC-SR04 x4) ======================
#define ULTRA_FRONT_TRIG_PIN    26
#define ULTRA_FRONT_ECHO_PIN    27
#define ULTRA_BACK_TRIG_PIN     0
#define ULTRA_BACK_ECHO_PIN     1
#define ULTRA_LEFT_TRIG_PIN     6
#define ULTRA_LEFT_ECHO_PIN     7
#define ULTRA_RIGHT_TRIG_PIN    2
#define ULTRA_RIGHT_ECHO_PIN    3

// ====================== ToF (3x VL53L1X, shared I2C0) ======================
#define TOF_LEFT_XSHUT   18
#define TOF_FRONT_XSHUT  19
#define TOF_RIGHT_XSHUT  20

#define TOF_LEFT_ADDR    0x30
#define TOF_FRONT_ADDR   0x31
#define TOF_RIGHT_ADDR   0x32

#define TOF_DEFAULT_ADDR 0x29

#endif // SENSOR_PINS_H
