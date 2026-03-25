#include "AS5600.h"

#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "AS5600";

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
AS5600::AS5600(i2c_port_t port, int sda_pin, int scl_pin,
               uint32_t freq_hz, uint32_t timeout)
    : _port(port), _sda(sda_pin), _scl(scl_pin),
      _freq(freq_hz), _timeout_ms(timeout),
      _prev_angle_deg(0.0f), _prev_time_us(0), _velocity_ready(false)
{}

// ─────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::begin()
{
    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = _sda;
    conf.scl_io_num       = _scl;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = _freq;

    esp_err_t err = i2c_param_config(_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    // Verify comms by reading the STATUS register
    uint8_t status = 0;
    err = getStatus(&status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor not responding – check wiring/address");
        return err;
    }

    ESP_LOGI(TAG, "AS5600 ready on I2C%d  SDA=%d SCL=%d  %lu Hz",
             _port, _sda, _scl, (unsigned long)_freq);
    return ESP_OK;
}

esp_err_t AS5600::end()
{
    return i2c_driver_delete(_port);
}

// ─────────────────────────────────────────────────────────────
//  Private low-level I2C helpers
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::_read(uint8_t reg, uint8_t *data, size_t len) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t AS5600::_write(uint8_t reg, const uint8_t *data, size_t len) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t AS5600::_read16(uint8_t reg_h, uint16_t *out) const
{
    uint8_t buf[2] = {0};
    esp_err_t err = _read(reg_h, buf, 2);
    if (err == ESP_OK) {
        *out = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }
    return err;
}

esp_err_t AS5600::_write16(uint8_t reg_h, uint16_t value) const
{
    uint8_t buf[2] = {
        (uint8_t)((value >> 8) & 0x0F),
        (uint8_t)(value & 0xFF)
    };
    return _write(reg_h, buf, 2);
}

esp_err_t AS5600::_modifyConf(uint16_t mask, uint16_t value)
{
    uint16_t conf = 0;
    esp_err_t err = getConf(&conf);
    if (err != ESP_OK) return err;
    conf = (conf & ~mask) | (value & mask);
    return setConf(conf);
}

// ─────────────────────────────────────────────────────────────
//  Static helpers
// ─────────────────────────────────────────────────────────────
float AS5600::_rawToDeg(uint16_t raw)
{
    return (raw / AS5600_RAW_MAX) * 360.0f;
}

float AS5600::_rawToRad(uint16_t raw)
{
    return (raw / AS5600_RAW_MAX) * 2.0f * (float)M_PI;
}

/**
 * Wrap an angular delta into (-180, +180] to handle rollover correctly.
 */
float AS5600::_wrapDelta(float delta)
{
    while (delta >  180.0f) delta -= 360.0f;
    while (delta <= -180.0f) delta += 360.0f;
    return delta;
}

// ─────────────────────────────────────────────────────────────
//  Status / diagnostics
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getStatus(uint8_t *status) const
{
    return _read(AS5600_REG_STATUS, status, 1);
}

esp_err_t AS5600::getMagnetStatus(as5600_magnet_status_t *out) const
{
    uint8_t status = 0;
    esp_err_t err = getStatus(&status);
    if (err == ESP_OK) {
        out->detected   = (status & AS5600_STATUS_MD) != 0;
        out->too_weak   = (status & AS5600_STATUS_ML) != 0;
        out->too_strong = (status & AS5600_STATUS_MH) != 0;
    }
    return err;
}

bool AS5600::isMagnetDetected() const
{
    as5600_magnet_status_t s = {};
    if (getMagnetStatus(&s) != ESP_OK) return false;
    return s.detected && !s.too_weak && !s.too_strong;
}

esp_err_t AS5600::getAGC(uint8_t *agc) const
{
    return _read(AS5600_REG_AGC, agc, 1);
}

esp_err_t AS5600::getMagnitude(uint16_t *mag) const
{
    uint8_t buf[2] = {0};
    esp_err_t err = _read(AS5600_REG_MAGNITUDE_H, buf, 2);
    if (err == ESP_OK) {
        *mag = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }
    return err;
}

// ─────────────────────────────────────────────────────────────
//  Raw angle
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getRawAngle(uint16_t *raw) const
{
    return _read16(AS5600_REG_RAW_ANGLE_H, raw);
}

esp_err_t AS5600::getAngle(uint16_t *angle) const
{
    return _read16(AS5600_REG_ANGLE_H, angle);
}

// ─────────────────────────────────────────────────────────────
//  Converted angle
// ─────────────────────────────────────────────────────────────
float AS5600::getAngleDeg() const
{
    uint16_t raw = 0;
    return (getAngle(&raw) == ESP_OK) ? _rawToDeg(raw) : NAN;
}

float AS5600::getAngleRad() const
{
    uint16_t raw = 0;
    return (getAngle(&raw) == ESP_OK) ? _rawToRad(raw) : NAN;
}

float AS5600::getRawAngleDeg() const
{
    uint16_t raw = 0;
    return (getRawAngle(&raw) == ESP_OK) ? _rawToDeg(raw) : NAN;
}

float AS5600::getRawAngleRad() const
{
    uint16_t raw = 0;
    return (getRawAngle(&raw) == ESP_OK) ? _rawToRad(raw) : NAN;
}

// ─────────────────────────────────────────────────────────────
//  Velocity
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getVelocityDegS(float *vel_deg_s)
{
    uint16_t raw = 0;
    esp_err_t err = getAngle(&raw);
    if (err != ESP_OK) return err;

    float now_deg  = _rawToDeg(raw);
    int64_t now_us = esp_timer_get_time();

    if (!_velocity_ready) {
        _prev_angle_deg  = now_deg;
        _prev_time_us    = now_us;
        _velocity_ready  = true;
        return ESP_ERR_INVALID_STATE;   // need two samples
    }

    float dt_s = (now_us - _prev_time_us) * 1e-6f;
    if (dt_s < 1e-6f) {
        return ESP_ERR_INVALID_STATE;   // avoid division by zero
    }

    float delta  = _wrapDelta(now_deg - _prev_angle_deg);
    *vel_deg_s   = delta / dt_s;

    _prev_angle_deg = now_deg;
    _prev_time_us   = now_us;
    return ESP_OK;
}

esp_err_t AS5600::getVelocityRadS(float *vel_rad_s)
{
    float v = 0.0f;
    esp_err_t err = getVelocityDegS(&v);
    if (err == ESP_OK) {
        *vel_rad_s = v * (float)M_PI / 180.0f;
    }
    return err;
}

// ─────────────────────────────────────────────────────────────
//  Zero position
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getZeroPosition(uint16_t *zpos) const
{
    return _read16(AS5600_REG_ZPOS_H, zpos);
}

esp_err_t AS5600::setZeroPosition(uint16_t zpos)
{
    return _write16(AS5600_REG_ZPOS_H, zpos & 0x0FFF);
}

esp_err_t AS5600::setZeroHere()
{
    uint16_t raw = 0;
    esp_err_t err = getRawAngle(&raw);
    if (err != ESP_OK) return err;
    return setZeroPosition(raw);
}

// ─────────────────────────────────────────────────────────────
//  Max position
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getMaxPosition(uint16_t *mpos) const
{
    return _read16(AS5600_REG_MPOS_H, mpos);
}

esp_err_t AS5600::setMaxPosition(uint16_t mpos)
{
    return _write16(AS5600_REG_MPOS_H, mpos & 0x0FFF);
}

// ─────────────────────────────────────────────────────────────
//  Max angle
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getMaxAngle(uint16_t *mang) const
{
    return _read16(AS5600_REG_MANG_H, mang);
}

esp_err_t AS5600::setMaxAngle(uint16_t mang)
{
    return _write16(AS5600_REG_MANG_H, mang & 0x0FFF);
}

// ─────────────────────────────────────────────────────────────
//  Configuration register
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::getConf(uint16_t *conf) const
{
    uint8_t buf[2] = {0};
    // CONF_H is at 0x07, CONF_L at 0x08 – full 16 bits, no 4-bit mask
    esp_err_t err = _read(AS5600_REG_CONF_H, buf, 2);
    if (err == ESP_OK) {
        *conf = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return err;
}

esp_err_t AS5600::setConf(uint16_t conf)
{
    uint8_t buf[2] = {
        (uint8_t)((conf >> 8) & 0x3F),   // upper 6 bits (bits 13:8)
        (uint8_t)(conf & 0xFF)            // lower 8 bits
    };
    return _write(AS5600_REG_CONF_H, buf, 2);
}

esp_err_t AS5600::getConfDecoded(as5600_conf_t *out) const
{
    uint16_t raw = 0;
    esp_err_t err = getConf(&raw);
    if (err != ESP_OK) return err;

    out->power_mode   = (raw >> 0)  & 0x03;
    out->hysteresis   = (raw >> 2)  & 0x03;
    out->output_stage = (raw >> 4)  & 0x03;
    out->pwm_freq     = (raw >> 6)  & 0x03;
    out->slow_filter  = (raw >> 8)  & 0x03;
    out->fth          = (raw >> 10) & 0x07;
    out->watchdog     = (raw >> 13) & 0x01;
    return ESP_OK;
}

esp_err_t AS5600::setConfDecoded(const as5600_conf_t *cfg)
{
    uint16_t raw = 0;
    raw |= (uint16_t)(cfg->power_mode   & 0x03) << 0;
    raw |= (uint16_t)(cfg->hysteresis   & 0x03) << 2;
    raw |= (uint16_t)(cfg->output_stage & 0x03) << 4;
    raw |= (uint16_t)(cfg->pwm_freq     & 0x03) << 6;
    raw |= (uint16_t)(cfg->slow_filter  & 0x03) << 8;
    raw |= (uint16_t)(cfg->fth          & 0x07) << 10;
    raw |= (uint16_t)(cfg->watchdog     & 0x01) << 13;
    return setConf(raw);
}

esp_err_t AS5600::setPowerMode(uint8_t pm)
{
    return _modifyConf(0x0003, (uint16_t)(pm & 0x03));
}

esp_err_t AS5600::setHysteresis(uint8_t hyst)
{
    return _modifyConf(0x000C, (uint16_t)(hyst & 0x03) << 2);
}

esp_err_t AS5600::setSlowFilter(uint8_t sf)
{
    return _modifyConf(0x0300, (uint16_t)(sf & 0x03) << 8);
}

esp_err_t AS5600::setFastFilterThreshold(uint8_t fth)
{
    return _modifyConf(0x1C00, (uint16_t)(fth & 0x07) << 10);
}

esp_err_t AS5600::setWatchdog(bool enable)
{
    return _modifyConf(0x2000, enable ? 0x2000 : 0x0000);
}

// ─────────────────────────────────────────────────────────────
//  Burn (OTP)
// ─────────────────────────────────────────────────────────────
esp_err_t AS5600::burnAngle()
{
    if (!isMagnetDetected()) {
        ESP_LOGE(TAG, "burnAngle: magnet not detected – aborting");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t cmd = AS5600_BURN_ANGLE;
    return _write(AS5600_REG_BURN, &cmd, 1);
}

esp_err_t AS5600::burnSettings()
{
    uint8_t cmd = AS5600_BURN_SETTINGS;
    return _write(AS5600_REG_BURN, &cmd, 1);
}

esp_err_t AS5600::getBurnCount(uint8_t *count) const
{
    return _read(AS5600_REG_ZMCO, count, 1);
}

// ─────────────────────────────────────────────────────────────
//  Debug
// ─────────────────────────────────────────────────────────────
void AS5600::printStatus() const
{
    printf("=== AS5600 Status ===\n");

    // Magnet
    as5600_magnet_status_t ms = {};
    if (getMagnetStatus(&ms) == ESP_OK) {
        printf("  Magnet detected : %s\n", ms.detected   ? "YES" : "NO");
        printf("  Magnet too weak : %s\n", ms.too_weak   ? "YES" : "NO");
        printf("  Magnet too strong: %s\n", ms.too_strong ? "YES" : "NO");
    }

    // AGC
    uint8_t agc = 0;
    if (getAGC(&agc) == ESP_OK)
        printf("  AGC             : %u\n", agc);

    // Magnitude
    uint16_t mag = 0;
    if (getMagnitude(&mag) == ESP_OK)
        printf("  Magnitude       : %u\n", mag);

    // Angle
    uint16_t raw_a = 0, filt_a = 0;
    if (getRawAngle(&raw_a) == ESP_OK)
        printf("  Raw angle       : %u  (%.2f deg)\n", raw_a, _rawToDeg(raw_a));
    if (getAngle(&filt_a) == ESP_OK)
        printf("  Filtered angle  : %u  (%.2f deg)\n", filt_a, _rawToDeg(filt_a));

    // ZPOS / MPOS / MANG
    uint16_t zpos = 0, mpos = 0, mang = 0;
    if (getZeroPosition(&zpos) == ESP_OK)
        printf("  ZPOS            : %u  (%.2f deg)\n", zpos, _rawToDeg(zpos));
    if (getMaxPosition(&mpos) == ESP_OK)
        printf("  MPOS            : %u  (%.2f deg)\n", mpos, _rawToDeg(mpos));
    if (getMaxAngle(&mang) == ESP_OK)
        printf("  MANG            : %u  (%.2f deg)\n", mang, _rawToDeg(mang));

    // CONF
    as5600_conf_t cfg = {};
    if (getConfDecoded(&cfg) == ESP_OK) {
        printf("  Power mode      : %u\n", cfg.power_mode);
        printf("  Hysteresis      : %u\n", cfg.hysteresis);
        printf("  Output stage    : %u\n", cfg.output_stage);
        printf("  PWM frequency   : %u\n", cfg.pwm_freq);
        printf("  Slow filter     : %u\n", cfg.slow_filter);
        printf("  Fast filter thr : %u\n", cfg.fth);
        printf("  Watchdog        : %s\n", cfg.watchdog ? "ON" : "OFF");
    }

    // Burn count
    uint8_t zmco = 0;
    if (getBurnCount(&zmco) == ESP_OK)
        printf("  Burn count      : %u / 3\n", zmco);

    printf("=====================\n");
}