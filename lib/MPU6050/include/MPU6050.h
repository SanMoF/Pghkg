#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#define MPU6050_I2C_ADDR        0x68

#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_WHO_AM_I    0x75

// Default full-scale sensitivities (±2 g / ±250 deg/s, the power-on defaults).
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_LSB_PER_DPS  131.0f

/**
 * @brief Driver for the InvenSense MPU6050 6-axis IMU (I2C), producing a
 *        complementary-filtered pitch angle suitable for a 2-wheel balance
 *        control loop.
 *
 * Uses the new ESP-IDF I2C master driver (driver/i2c_master.h). It cannot
 * coexist in one firmware with the legacy driver/i2c.h (AS5600, TCS34725,
 * i2c_lcd libs) — IDF aborts at boot if both are linked in.
 *
 * Usage:
 * @code
 *   MPU6050 imu(I2C_NUM_0, 21, 22);
 *   imu.begin();
 *   imu.calibrateGyro();      // robot must be still
 *   // in the control loop, called every dt seconds:
 *   imu.update(dt);
 *   float pitch = imu.getPitchDeg();
 * @endcode
 */
class MPU6050
{
public:
    // freq_hz defaults to 100kHz (not the MPU6050's max 400kHz) because
    // breadboard/jumper-wire I2C runs are prone to noise and clock-stretch
    // glitches at 400kHz that show up as total silence (every transaction
    // NACKs) rather than a clean speed-limited error. Drop to 100kHz first
    // when debugging "nothing responds" before suspecting the wiring itself.
    // timeout is per transaction, in ms. Kept short (was 1000) because
    // update() runs inside the 5 ms control loop: a stuck bus must fail fast
    // instead of freezing the loop for a second.
    MPU6050(i2c_port_num_t port    = I2C_NUM_0,
            int            sda_pin = 21,
            int            scl_pin = 22,
            uint32_t       freq_hz = 100000,
            uint32_t       timeout = 20);

    ~MPU6050();
    MPU6050(const MPU6050 &) = delete;
    MPU6050 &operator=(const MPU6050 &) = delete;

    // Initialise I2C, wake the sensor and verify WHO_AM_I.
    esp_err_t begin();

    // Average `samples` gyro readings while the robot is stationary to
    // remove the Y-axis (pitch) gyro bias. Blocking — call once from setup.
    esp_err_t calibrateGyro(int samples = 200);

    // Read raw accel/gyro registers (6 int16 each, in that order).
    esp_err_t readRaw(int16_t accel[3], int16_t gyro[3]) const;

    // Debug helper: probes every 7-bit I2C address (1-126) on this bus and
    // prints which ones ACK. Call after begin() if WHO_AM_I fails, to check
    // whether the MPU6050 is on the bus at all (wrong address / no ACK at
    // all usually means wiring, power, or missing pull-ups).
    void scanBus() const;

    // Complementary filter step — call once per control-loop tick with the
    // loop's fixed period in seconds (must match the caller's timer period).
    esp_err_t update(float dt_s);

    // Latest filtered pitch angle in degrees from the last update() call.
    float getPitchDeg() const { return _pitch_deg; }

private:
    i2c_port_num_t          _port;
    int                     _sda;
    int                     _scl;
    uint32_t                _freq;
    int                     _timeout_ms;
    i2c_master_bus_handle_t _bus;
    i2c_master_dev_handle_t _dev;

    float _gyro_offset_dps;
    float _pitch_deg;
    bool  _pitch_ready;

    esp_err_t _read(uint8_t reg, uint8_t *data, size_t len) const;
    esp_err_t _write(uint8_t reg, uint8_t value) const;
};
