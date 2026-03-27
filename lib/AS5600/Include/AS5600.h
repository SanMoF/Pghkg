#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// ─────────────────────────────────────────────────────────────
//  Register map (AS5600 datasheet Rev 1.9)
// ─────────────────────────────────────────────────────────────
#define AS5600_I2C_ADDR         0x36

// Configuration
#define AS5600_REG_ZMCO         0x00
#define AS5600_REG_ZPOS_H       0x01
#define AS5600_REG_ZPOS_L       0x02
#define AS5600_REG_MPOS_H       0x03
#define AS5600_REG_MPOS_L       0x04
#define AS5600_REG_MANG_H       0x05
#define AS5600_REG_MANG_L       0x06
#define AS5600_REG_CONF_H       0x07
#define AS5600_REG_CONF_L       0x08

// Output
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_RAW_ANGLE_L  0x0D
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_ANGLE_L      0x0F

// Diagnostics
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE_H  0x1B
#define AS5600_REG_MAGNITUDE_L  0x1C

// Burn command
#define AS5600_REG_BURN         0xFF

// STATUS bits
#define AS5600_STATUS_MD        (1 << 5)   ///< Magnet detected
#define AS5600_STATUS_ML        (1 << 4)   ///< Magnet too low  (weak)
#define AS5600_STATUS_MH        (1 << 3)   ///< Magnet too high (strong)

// CONF register – Power Mode (bits 1:0)
#define AS5600_PM_NOM           0x00   ///< Normal (polling)
#define AS5600_PM_LPM1          0x01   ///< Low power 1
#define AS5600_PM_LPM2          0x02   ///< Low power 2
#define AS5600_PM_LPM3          0x03   ///< Low power 3

// CONF register – Hysteresis (bits 3:2)
#define AS5600_HYST_OFF         0x00
#define AS5600_HYST_1LSB        0x01
#define AS5600_HYST_2LSB        0x02
#define AS5600_HYST_3LSB        0x03

// CONF register – Output Stage (bits 5:4)
#define AS5600_OUT_ANALOG_FULL  0x00   ///< Analog  0 – 100 %  VDD
#define AS5600_OUT_ANALOG_RED   0x01   ///< Analog 10 – 90  %  VDD
#define AS5600_OUT_PWM          0x02   ///< PWM

// CONF register – PWM Frequency (bits 7:6)
#define AS5600_PWM_115HZ        0x00
#define AS5600_PWM_230HZ        0x01
#define AS5600_PWM_460HZ        0x02
#define AS5600_PWM_920HZ        0x03

// CONF register – Slow Filter (bits 9:8)
#define AS5600_SF_16X           0x00
#define AS5600_SF_8X            0x01
#define AS5600_SF_4X            0x02
#define AS5600_SF_2X            0x03

// CONF register – Fast Filter Threshold (bits 12:10)
#define AS5600_FTH_SLOW_ONLY    0x00
#define AS5600_FTH_6LSB         0x01
#define AS5600_FTH_7LSB         0x02
#define AS5600_FTH_9LSB         0x03
#define AS5600_FTH_18LSB        0x04
#define AS5600_FTH_21LSB        0x05
#define AS5600_FTH_24LSB        0x06
#define AS5600_FTH_10LSB        0x07

// CONF register – Watchdog (bit 13)
#define AS5600_WD_OFF           0x00
#define AS5600_WD_ON            0x01

// Burn commands
#define AS5600_BURN_ANGLE       0x80
#define AS5600_BURN_SETTINGS    0x40

// Resolution
#define AS5600_RAW_MAX          4096.0f   ///< 12-bit (0 – 4095)

// ─────────────────────────────────────────────────────────────
//  Structs
// ─────────────────────────────────────────────────────────────

/** Magnet status decoded from STATUS register */
typedef struct {
    bool detected;    ///< MD bit – magnet in range
    bool too_weak;    ///< ML bit – flux too low
    bool too_strong;  ///< MH bit – flux too high
} as5600_magnet_status_t;

/** Full configuration word (16-bit CONF register) */
typedef struct {
    uint8_t power_mode;    ///< AS5600_PM_*
    uint8_t hysteresis;    ///< AS5600_HYST_*
    uint8_t output_stage;  ///< AS5600_OUT_*
    uint8_t pwm_freq;      ///< AS5600_PWM_*
    uint8_t slow_filter;   ///< AS5600_SF_*
    uint8_t fth;           ///< AS5600_FTH_*
    uint8_t watchdog;      ///< AS5600_WD_*
} as5600_conf_t;

// ─────────────────────────────────────────────────────────────
//  Class
// ─────────────────────────────────────────────────────────────

/**
 * @brief Driver for the AMS AS5600 12-bit magnetic rotary encoder.
 *
 * Usage example:
 * @code
 *   AS5600 enc(I2C_NUM_0, 21, 22);
 *   enc.begin();
 *   float deg = enc.getAngleDeg();
 * @endcode
 */
class AS5600 {
public:
    /**
     * @param port      I2C port number (I2C_NUM_0 or I2C_NUM_1)
     * @param sda_pin   GPIO number for SDA
     * @param scl_pin   GPIO number for SCL
     * @param freq_hz   I2C clock frequency in Hz (default 400 000)
     * @param timeout   I2C transaction timeout in ms (default 1000)
     */
    AS5600(i2c_port_t port   = I2C_NUM_0,
           int        sda_pin = 21,
           int        scl_pin = 22,
           uint32_t   freq_hz = 400000,
           uint32_t   timeout = 1000);

    // ── Lifecycle ───────────────────────────────────────────
    /**
     * @brief Initialise I2C and verify the sensor is reachable.
     * @return ESP_OK on success.
     */
    esp_err_t begin();

    /**
     * @brief Release the I2C driver (call on teardown).
     */
    esp_err_t end();

    // ── Status / diagnostics ────────────────────────────────
    /**
     * @brief Read raw STATUS register byte.
     */
    esp_err_t getStatus(uint8_t *status) const;

    /**
     * @brief Decode STATUS byte into a friendly struct.
     */
    esp_err_t getMagnetStatus(as5600_magnet_status_t *out) const;

    /**
     * @brief Returns true if a magnet is properly detected (MD=1, ML=0, MH=0).
     */
    bool isMagnetDetected() const;

    /**
     * @brief Automatic Gain Control value (0–255). Lower = stronger magnet.
     */
    esp_err_t getAGC(uint8_t *agc) const;

    /**
     * @brief Internal CORDIC magnitude (12-bit).
     */
    esp_err_t getMagnitude(uint16_t *mag) const;

    // ── Raw angle ───────────────────────────────────────────
    /**
     * @brief Unfiltered 12-bit angle (0x0C–0x0D). Range 0–4095.
     */
    esp_err_t getRawAngle(uint16_t *raw) const;

    /**
     * @brief Filtered & scaled 12-bit angle (0x0E–0x0F). Range 0–4095.
     */
    esp_err_t getAngle(uint16_t *angle) const;

    // ── Converted angle ─────────────────────────────────────
    /**
     * @brief Filtered angle in degrees [0.0, 360.0).
     *        Returns NAN on I2C error.
     */
    float getAngleDeg() const;

    /**
     * @brief Filtered angle in radians [0, 2π).
     *        Returns NAN on I2C error.
     */
    float getAngleRad() const;

    /**
     * @brief Unfiltered raw angle in degrees [0.0, 360.0).
     */
    float getRawAngleDeg() const;

    /**
     * @brief Unfiltered raw angle in radians [0, 2π).
     */
    float getRawAngleRad() const;
    // Método público nuevo
float getAccumulatedAngleDeg();  // Ángulo global (puede superar ±360°)
void  resetAccumulatedAngle();   // Resetear a cero desde posición actual

    // ── Velocity ────────────────────────────────────────────
    /**
     * @brief Compute angular velocity (deg/s) from two successive reads.
     *
     * Call this method repeatedly in a loop; it internally tracks the
     * previous angle and timestamp using esp_timer_get_time().
     *
     * @param[out] vel_deg_s  Angular velocity in degrees per second.
     * @return ESP_OK, or ESP_ERR_INVALID_STATE if called before the
     *         second reading is available.
     */
    esp_err_t getVelocityDegS(float *vel_deg_s);

    /**
     * @brief Same as getVelocityDegS but in rad/s.
     */
    esp_err_t getVelocityRadS(float *vel_rad_s);

    // ── Zero position (start angle) ─────────────────────────
    /**
     * @brief Read ZPOS register (12-bit).
     */
    esp_err_t getZeroPosition(uint16_t *zpos) const;

    /**
     * @brief Write ZPOS register (12-bit, 0–4095).
     *        Sets the zero/start angle for the output range.
     */
    esp_err_t setZeroPosition(uint16_t zpos);

    /**
     * @brief Convenience: set zero at the current magnet position.
     */
    esp_err_t setZeroHere();

    // ── Max position (stop angle) ───────────────────────────
    /**
     * @brief Read MPOS register (12-bit).
     */
    esp_err_t getMaxPosition(uint16_t *mpos) const;

    /**
     * @brief Write MPOS register (12-bit).
     *        Sets the stop angle for the output range.
     */
    esp_err_t setMaxPosition(uint16_t mpos);

    // ── Max angle (output range) ────────────────────────────
    /**
     * @brief Read MANG register (12-bit). Defines the angular range mapped
     *        to the full output. Minimum value is 18 degrees.
     */
    esp_err_t getMaxAngle(uint16_t *mang) const;

    /**
     * @brief Write MANG register (12-bit).
     */
    esp_err_t setMaxAngle(uint16_t mang);

    // ── Configuration register ──────────────────────────────
    /**
     * @brief Read raw 16-bit CONF register.
     */
    esp_err_t getConf(uint16_t *conf) const;

    /**
     * @brief Write raw 16-bit CONF register.
     */
    esp_err_t setConf(uint16_t conf);

    /**
     * @brief Read CONF register decoded into as5600_conf_t.
     */
    esp_err_t getConfDecoded(as5600_conf_t *out) const;

    /**
     * @brief Encode as5600_conf_t and write to CONF register.
     */
    esp_err_t setConfDecoded(const as5600_conf_t *cfg);

    /**
     * @brief Set power mode (AS5600_PM_*).
     */
    esp_err_t setPowerMode(uint8_t pm);

    /**
     * @brief Set hysteresis (AS5600_HYST_*).
     */
    esp_err_t setHysteresis(uint8_t hyst);

    /**
     * @brief Set slow filter (AS5600_SF_*).
     */
    esp_err_t setSlowFilter(uint8_t sf);

    /**
     * @brief Set fast filter threshold (AS5600_FTH_*).
     */
    esp_err_t setFastFilterThreshold(uint8_t fth);

    /**
     * @brief Enable or disable the watchdog timer.
     */
    esp_err_t setWatchdog(bool enable);

    // ── Burn (OTP, IRREVERSIBLE) ────────────────────────────
    /**
     * @brief Burn current angle settings into OTP (max 3 times, IRREVERSIBLE).
     *        A magnet must be detected before calling this.
     */
    esp_err_t burnAngle();

    /**
     * @brief Burn current CONF + MANG into OTP (once only, IRREVERSIBLE).
     */
    esp_err_t burnSettings();

    /**
     * @brief Number of times the angle OTP has been burned (0–3).
     */
    esp_err_t getBurnCount(uint8_t *count) const;

    // ── Debug helper ─────────────────────────────────────────
    /**
     * @brief Print a formatted status summary via printf.
     */
    void printStatus() const;

private:
    

    i2c_port_t  _port;
    int         _sda;
    int         _scl;
    uint32_t    _freq;
    uint32_t    _timeout_ms;

    // Accumulated angle
    float   _last_angle_deg;
    float   _accumulated_deg;
    bool    _accum_ready;

    // Velocity tracking
    float   _prev_angle_deg;
    int64_t _prev_time_us;
    bool    _velocity_ready;
    // Low-level I2C
    esp_err_t _read(uint8_t reg, uint8_t *data, size_t len) const;
    esp_err_t _write(uint8_t reg, const uint8_t *data, size_t len) const;
    esp_err_t _read16(uint8_t reg_h, uint16_t *out) const;
    esp_err_t _write16(uint8_t reg_h, uint16_t value) const;
    esp_err_t _modifyConf(uint16_t mask, uint16_t value);

    static float _rawToDeg(uint16_t raw);
    static float _rawToRad(uint16_t raw);
    static float _wrapDelta(float delta);
};