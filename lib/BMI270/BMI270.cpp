#include "BMI270.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmi270_config_file.h"

static const char *TAG = "BMI270";

// ── Register map (BMI270 datasheet) ─────────────────────────────────
#define REG_CHIP_ID          0x00
#define REG_ACC_X_LSB        0x0C   // 12-byte burst covers accel+gyro X/Y/Z
#define REG_INTERNAL_STATUS  0x21
#define REG_ACC_CONF         0x40
#define REG_ACC_RANGE        0x41
#define REG_GYR_CONF         0x42
#define REG_GYR_RANGE        0x43
#define REG_INIT_CTRL        0x59
#define REG_INIT_ADDR_0      0x5B
#define REG_INIT_ADDR_1      0x5C
#define REG_INIT_DATA        0x5E
#define REG_PWR_CONF         0x7C
#define REG_PWR_CTRL         0x7D
#define REG_CMD              0x7E

#define CMD_SOFT_RESET       0xB6

// Complementary filter blend — trust the integrated gyro rate most of the
// time, correct slow drift with the accelerometer's absolute angle.
static const float COMP_FILTER_ALPHA = 0.98f;

// ── Anti-vibration low-pass filters ─────────────────────────────────
// The accel and gyro ODRs below must match REG_ACC_CONF / REG_GYR_CONF in
// begin() (100 Hz / 200 Hz) — the biquad coefficients are derived from
// these sample rates, so changing one without the other desyncs the filter.
static const float ACCEL_FILTER_FS_HZ = 100.0f;
static const float GYRO_FILTER_FS_HZ  = 200.0f;
static const float VIBRATION_FILTER_FC_HZ = 20.0f;

// 2nd-order Butterworth low-pass biquad (RBJ Audio EQ Cookbook, Q=1/sqrt(2)),
// bilinear-transformed at the given sample rate. b/a already normalized by
// a0, so a[0] = 1.0 and Filter::apply() can use them directly.
static void computeButterworthLPF(float fc_hz, float fs_hz, float b[3], float a[3])
{
    float w0    = 2.0f * (float)M_PI * fc_hz / fs_hz;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / (2.0f * 0.70710678f); // Q = 1/sqrt(2) => Butterworth

    float a0 = 1.0f + alpha;
    b[0] = ((1.0f - cosw0) / 2.0f) / a0;
    b[1] = (1.0f - cosw0) / a0;
    b[2] = b[0];
    a[0] = 1.0f;
    a[1] = (-2.0f * cosw0) / a0;
    a[2] = (1.0f - alpha) / a0;
}

BMI270::BMI270(i2c_port_t port, int sda_pin, int scl_pin,
               uint32_t freq_hz, uint8_t addr)
    : _port(port), _sda(sda_pin), _scl(scl_pin),
      _freq(freq_hz), _addr(addr), _timeout_ms(20),
      _gyro_offset_dps(0.0f), _pitch_deg(0.0f), _pitch_ready(false)
{
    float b[3], a[3];

    computeButterworthLPF(VIBRATION_FILTER_FC_HZ, ACCEL_FILTER_FS_HZ, b, a);
    _accelPitchFilter.setup(b, a, 3, 3, 0.0f, 0.0f); // robot assumed level at boot

    computeButterworthLPF(VIBRATION_FILTER_FC_HZ, GYRO_FILTER_FS_HZ, b, a);
    _gyroRateFilter.setup(b, a, 3, 3, 0.0f, 0.0f); // robot assumed still at boot
}

esp_err_t BMI270::_readReg(uint8_t reg, uint8_t *data, size_t len) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t BMI270::_writeReg(uint8_t reg, uint8_t value) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t BMI270::_writeBurst(uint8_t reg, const uint8_t *data, size_t len) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);

    // Config-file writes are larger than a normal register poke; give them
    // more room than the default register timeout.
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms + 20));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t BMI270::_loadConfigFile()
{
    if (bmi270_config_file_len == 0) {
        printf("[BMI270] Config file array is empty. Paste Bosch's official "
               "bmi270_config_file[] (from BMI270_SensorAPI's bmi270.c) into "
               "lib/BMI270/bmi270_config_file.cpp before this sensor can "
               "initialize — see bmi270_config_file.h for details.\n");
        return ESP_ERR_NOT_FOUND;
    }

    // Disable advanced power save so the config RAM is reachable, then wait
    // the datasheet's settling time (>450 us) before touching INIT_CTRL.
    esp_err_t err = _writeReg(REG_PWR_CONF, 0x00);
    if (err != ESP_OK) {
        printf("[BMI270] PWR_CONF write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // INIT_CTRL = 0x00 arms the loader.
    err = _writeReg(REG_INIT_CTRL, 0x00);
    if (err != ESP_OK) {
        printf("[BMI270] INIT_CTRL(arm) write failed: %s\n", esp_err_to_name(err));
        return err;
    }

    // Burst-write the config file in 32-byte chunks (matches Bosch's
    // reference driver's I2C read_write_len). INIT_ADDR_0/1 hold the write
    // pointer in 16-bit-word units (byte_offset / 2): addr_0 gets the LOW
    // 4 bits, addr_1 gets the rest — verified against Bosch's own
    // BMI270_SensorAPI (bmi2.c, upload_file()), NOT the byte split you'd
    // guess from a generic "16-bit address register" assumption.
    const size_t CHUNK = 32;
    for (size_t offset = 0; offset < bmi270_config_file_len; offset += CHUNK) {
        size_t len = CHUNK;
        if (offset + len > bmi270_config_file_len)
            len = bmi270_config_file_len - offset;

        uint16_t index = (uint16_t)(offset / 2);
        err = _writeReg(REG_INIT_ADDR_0, (uint8_t)(index & 0x0F));
        if (err != ESP_OK) {
            printf("[BMI270] INIT_ADDR_0 write failed at offset %u: %s\n",
                   (unsigned)offset, esp_err_to_name(err));
            return err;
        }
        err = _writeReg(REG_INIT_ADDR_1, (uint8_t)(index >> 4));
        if (err != ESP_OK) {
            printf("[BMI270] INIT_ADDR_1 write failed at offset %u: %s\n",
                   (unsigned)offset, esp_err_to_name(err));
            return err;
        }
        err = _writeBurst(REG_INIT_DATA, &bmi270_config_file[offset], len);
        if (err != ESP_OK) {
            printf("[BMI270] INIT_DATA burst write failed at offset %u (len %u): %s\n",
                   (unsigned)offset, (unsigned)len, esp_err_to_name(err));
            return err;
        }
    }

    // INIT_CTRL = 0x01 signals "done loading" and applies the config file.
    err = _writeReg(REG_INIT_CTRL, 0x01);
    if (err != ESP_OK) {
        printf("[BMI270] INIT_CTRL(apply) write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // datasheet: up to 20 ms to apply

    uint8_t status = 0;
    err = _readReg(REG_INTERNAL_STATUS, &status, 1);
    if (err != ESP_OK) {
        printf("[BMI270] INTERNAL_STATUS read failed: %s\n", esp_err_to_name(err));
        return err;
    }
    if ((status & 0x0F) != 0x01) {
        printf("[BMI270] Config load failed, INTERNAL_STATUS=0x%02X (expected 0x01)\n",
               status);
        return ESP_FAIL;
    }
    printf("[BMI270] Config file loaded OK, INTERNAL_STATUS=0x%02X\n", status);
    return ESP_OK;
}

esp_err_t BMI270::begin()
{
    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = (gpio_num_t)_sda;
    conf.scl_io_num       = (gpio_num_t)_scl;
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

    uint8_t chip_id = 0;
    err = _readReg(REG_CHIP_ID, &chip_id, 1);
    printf("[BMI270] CHIP_ID read at 0x%02X: err=%s value=0x%02X\n",
           _addr, esp_err_to_name(err), chip_id);
    if (err != ESP_OK || chip_id != BMI270_CHIP_ID_EXPECTED) {
        // The BMI270's address is set by its SDO/AD0 pin level (low=0x68,
        // high=0x69). A floating SDO pin can flip which address answers
        // between boots (seen on the bench: worked, was unplugged, came
        // back at the other address). Try the other address once before
        // giving up — but this only masks a loose/floating SDO connection,
        // it doesn't fix it; re-seat SDO to a firm GND or VCC.
        uint8_t alt_addr = (_addr == 0x68) ? 0x69 : 0x68;
        uint8_t orig_addr = _addr;
        _addr = alt_addr;
        err = _readReg(REG_CHIP_ID, &chip_id, 1);
        printf("[BMI270] Retry CHIP_ID read at 0x%02X: err=%s value=0x%02X\n",
               _addr, esp_err_to_name(err), chip_id);

        if (err == ESP_OK && chip_id == BMI270_CHIP_ID_EXPECTED) {
            printf("[BMI270] WARNING: found on 0x%02X instead of the configured "
                   "0x%02X — SDO/AD0 is likely floating. Wire it firmly to GND "
                   "(0x68) or VCC (0x69) instead of relying on this fallback.\n",
                   _addr, orig_addr);
        } else {
            _addr = orig_addr; // restore for the scan/error report below
            ESP_LOGE(TAG, "BMI270 not responding (CHIP_ID=0x%02X) — check wiring/address", chip_id);
            printf("[BMI270] Scanning I2C%d bus for any responding device...\n", _port);
            scanBus();
            return (err != ESP_OK) ? err : ESP_ERR_NOT_FOUND;
        }
    }

    // Soft reset, then wait for it to settle. Bosch's reference driver uses
    // 2 ms here, but that leaves this sensor NACKing the very next write
    // (confirmed on the bench) — 10 ms gives real margin over the
    // datasheet's minimum without meaningfully slowing down boot.
    err = _writeReg(REG_CMD, CMD_SOFT_RESET);
    if (err != ESP_OK) {
        printf("[BMI270] Soft reset write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    err = _loadConfigFile();
    if (err != ESP_OK) return err; // _loadConfigFile already printed the reason

    // Enable accel + gyro + temp in normal power mode.
    err = _writeReg(REG_PWR_CTRL, 0x0E);
    if (err != ESP_OK) {
        printf("[BMI270] PWR_CTRL write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Accel: ODR 100 Hz, normal filter, performance mode; range ±8 g.
    err = _writeReg(REG_ACC_CONF, 0xA8);
    if (err != ESP_OK) {
        printf("[BMI270] ACC_CONF write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    err = _writeReg(REG_ACC_RANGE, 0x02);
    if (err != ESP_OK) {
        printf("[BMI270] ACC_RANGE write failed: %s\n", esp_err_to_name(err));
        return err;
    }

    // Gyro: ODR 200 Hz, normal filter, high noise performance; range ±2000 dps.
    err = _writeReg(REG_GYR_CONF, 0xA9);
    if (err != ESP_OK) {
        printf("[BMI270] GYR_CONF write failed: %s\n", esp_err_to_name(err));
        return err;
    }
    err = _writeReg(REG_GYR_RANGE, 0x00);
    if (err != ESP_OK) {
        printf("[BMI270] GYR_RANGE write failed: %s\n", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BMI270 ready on I2C%d  SDA=%d SCL=%d  %lu Hz",
             _port, _sda, _scl, (unsigned long)_freq);
    return ESP_OK;
}

esp_err_t BMI270::readRaw(int16_t accel[3], int16_t gyro[3]) const
{
    uint8_t buf[12] = {0};
    esp_err_t err = _readReg(REG_ACC_X_LSB, buf, 12);
    if (err != ESP_OK)
        return err;

    // Little-endian, LSB first — opposite byte order from the MPU6050.
    accel[0] = (int16_t)((buf[1]  << 8) | buf[0]);
    accel[1] = (int16_t)((buf[3]  << 8) | buf[2]);
    accel[2] = (int16_t)((buf[5]  << 8) | buf[4]);
    gyro[0]  = (int16_t)((buf[7]  << 8) | buf[6]);
    gyro[1]  = (int16_t)((buf[9]  << 8) | buf[8]);
    gyro[2]  = (int16_t)((buf[11] << 8) | buf[10]);
    return ESP_OK;
}

void BMI270::scanBus() const
{
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("[BMI270] Device found at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("[BMI270] No I2C devices found on bus %d (SDA=%d SCL=%d). "
               "Nothing is ACKing -> check VCC/GND, SDA/SCL not swapped, "
               "and that the bus has pull-ups.\n",
               _port, _sda, _scl);
    } else {
        printf("[BMI270] %d device(s) found. If 0x68/0x69 is NOT in this "
               "list, the BMI270 itself isn't responding even though "
               "something else acks on the bus.\n", found);
    }
}

esp_err_t BMI270::calibrateGyro(int samples)
{
    if (samples <= 0)
        samples = 1;

    double sum = 0.0;
    int    ok  = 0;
    int16_t accel[3], gyro[3];

    for (int i = 0; i < samples; i++) {
        if (readRaw(accel, gyro) == ESP_OK) {
            sum += gyro[0]; // balance axis — see update()
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (ok == 0) {
        ESP_LOGE(TAG, "calibrateGyro: no successful reads");
        return ESP_FAIL;
    }

    _gyro_offset_dps = (float)(sum / ok) / BMI270_GYRO_LSB_PER_DPS;
    ESP_LOGI(TAG, "Gyro balance-axis offset: %.3f deg/s (%d/%d samples)", _gyro_offset_dps, ok, samples);
    return ESP_OK;
}

esp_err_t BMI270::update(float dt_s)
{
    int16_t accel[3], gyro[3];
    esp_err_t err = readRaw(accel, gyro);
    if (err != ESP_OK)
        return err;

    // Same axis pair confirmed on the bench for the MPU6050 on this robot's
    // mounting: ay/az track the forward/back fall, ax/az tracks left/right.
    // Sign not yet bench-verified for THIS sensor's orientation — confirm
    // with a debug print before arming the ESCs; flip the sign below if the
    // PID pushes the wrong way.
    float ay = (float)accel[1] / BMI270_ACCEL_LSB_PER_G;
    float az = (float)accel[2] / BMI270_ACCEL_LSB_PER_G;
    float accel_pitch_deg = atan2f(-ay, az) * 180.0f / (float)M_PI;
    accel_pitch_deg = _accelPitchFilter.apply(accel_pitch_deg);

    float gyro_rate_dps = (float)gyro[0] / BMI270_GYRO_LSB_PER_DPS - _gyro_offset_dps;
    gyro_rate_dps = _gyroRateFilter.apply(gyro_rate_dps);

    if (!_pitch_ready) {
        _pitch_deg   = accel_pitch_deg;
        _pitch_ready = true;
        return ESP_OK;
    }

    _pitch_deg = COMP_FILTER_ALPHA * (_pitch_deg + gyro_rate_dps * dt_s)
                 + (1.0f - COMP_FILTER_ALPHA) * accel_pitch_deg;

    return ESP_OK;
}
