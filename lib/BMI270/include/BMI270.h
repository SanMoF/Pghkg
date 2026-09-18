#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#define BMI270_CHIP_ID_EXPECTED  0x24

// Configured range: accel ±8g, gyro ±2000 dps (see BMI270::begin()).
// LSB/unit constants used to convert raw readRaw() counts to physical units.
#define BMI270_ACCEL_LSB_PER_G     4096.0f
#define BMI270_GYRO_LSB_PER_DPS    16.4f

/**
 * @brief Driver for the Bosch BMI270 6-DoF IMU (I2C), producing a
 *        complementary-filtered pitch angle suitable for a 2-wheel balance
 *        control loop. Same public shape as this project's MPU6050 driver
 *        (begin/calibrateGyro/update/readRaw/getPitchDeg) so it's a drop-in
 *        replacement in definitions.h.
 *
 * Usage:
 * @code
 *   BMI270 imu(I2C_NUM_0, 21, 22);
 *   imu.begin();
 *   imu.calibrateGyro();      // robot must be still
 *   // in the control loop, called every dt seconds:
 *   imu.update(dt);
 *   float pitch = imu.getPitchDeg();
 * @endcode
 *
 * IMPORTANT: begin() will fail (return ESP_ERR_NOT_FOUND) until Bosch's
 * official config-file array is pasted into bmi270_config_file.cpp — see
 * bmi270_config_file.h.
 */
class BMI270
{
public:
    // addr: 0x68 with the module's SDO pin pulled low (most breakouts'
    // default), 0x69 if SDO is pulled high.
    BMI270(i2c_port_t port    = I2C_NUM_0,
           int        sda_pin = 21,
           int        scl_pin = 22,
           uint32_t   freq_hz = 400000,
           uint8_t    addr    = 0x68);

    // Initialise I2C, soft-reset, upload Bosch's config file, verify
    // INTERNAL_STATUS, enable accel+gyro in normal mode.
    esp_err_t begin();

    // Average `samples` gyro readings while the robot is stationary to
    // remove the balance-axis gyro bias. Blocking — call once from setup.
    esp_err_t calibrateGyro(int samples = 200);

    // Read raw accel/gyro registers (3 int16 each, X/Y/Z order).
    esp_err_t readRaw(int16_t accel[3], int16_t gyro[3]) const;

    // Debug helper: probes every 7-bit I2C address (1-126) on this bus and
    // prints which ones ACK. Call after begin() if it fails, to check
    // whether the BMI270 is on the bus at all.
    void scanBus() const;

    // Complementary filter step — call once per control-loop tick with the
    // loop's fixed period in seconds (must match the caller's timer period).
    esp_err_t update(float dt_s);

    // Latest filtered pitch angle in degrees from the last update() call.
    float getPitchDeg() const { return _pitch_deg; }

private:
    i2c_port_t _port;
    int        _sda, _scl;
    uint32_t   _freq;
    uint8_t    _addr;
    uint32_t   _timeout_ms;

    float _gyro_offset_dps;
    float _pitch_deg;
    bool  _pitch_ready;

    esp_err_t _readReg(uint8_t reg, uint8_t *data, size_t len) const;
    esp_err_t _writeReg(uint8_t reg, uint8_t value) const;
    esp_err_t _writeBurst(uint8_t reg, const uint8_t *data, size_t len) const;
    esp_err_t _loadConfigFile();
};
