#include "MPU6050.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

// Complementary filter blend — trust the integrated gyro rate most of the
// time, correct slow drift with the accelerometer's absolute angle.
static const float COMP_FILTER_ALPHA = 0.98f;

MPU6050::MPU6050(i2c_port_t port, int sda_pin, int scl_pin,
                  uint32_t freq_hz, uint32_t timeout)
    : _port(port), _sda(sda_pin), _scl(scl_pin),
      _freq(freq_hz), _timeout_ms(timeout),
      _gyro_offset_dps(0.0f), _pitch_deg(0.0f), _pitch_ready(false)
{
}

esp_err_t MPU6050::_read(uint8_t reg, uint8_t *data, size_t len) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t MPU6050::_write(uint8_t reg, uint8_t value) const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(_timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t MPU6050::begin()
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

    uint8_t whoami = 0;
    err = _read(MPU6050_REG_WHO_AM_I, &whoami, 1);
    // Print the raw result unconditionally — on a healthy bus this reads
    // 0x68 (or 0x69 with AD0 pulled high); anything else (0x00, 0xFF, or an
    // ESP_ERR_TIMEOUT/ESP_FAIL) means the device never ACKed.
    printf("[MPU6050] WHO_AM_I read: err=%s value=0x%02X\n", esp_err_to_name(err), whoami);
    if (err != ESP_OK || (whoami & 0x7E) != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not responding (WHO_AM_I=0x%02X) — check wiring/address", whoami);
        printf("[MPU6050] Scanning I2C%d bus for any responding device...\n", _port);
        scanBus();
        return (err != ESP_OK) ? err : ESP_ERR_NOT_FOUND;
    }

    // Wake up: clear the SLEEP bit (device powers on asleep).
    err = _write(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake sensor: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU6050 ready on I2C%d  SDA=%d SCL=%d  %lu Hz",
             _port, _sda, _scl, (unsigned long)_freq);
    return ESP_OK;
}

esp_err_t MPU6050::readRaw(int16_t accel[3], int16_t gyro[3]) const
{
    uint8_t buf[14] = {0};
    esp_err_t err = _read(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (err != ESP_OK)
        return err;

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6],buf[7] = temperature — unused here.
    gyro[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
    return ESP_OK;
}

void MPU6050::scanBus() const
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
            printf("[MPU6050] Device found at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("[MPU6050] No I2C devices found on bus %d (SDA=%d SCL=%d). "
               "Nothing is ACKing -> check VCC/GND, SDA/SCL not swapped, "
               "and that the bus has pull-ups (external 4.7k, since a bare "
               "MPU6050 chip -- not a GY-521 breakout -- has none onboard).\n",
               _port, _sda, _scl);
    } else {
        printf("[MPU6050] %d device(s) found. If 0x68/0x69 is NOT in this "
               "list, the MPU6050 itself isn't responding even though "
               "something else acks on the bus.\n", found);
    }
}

esp_err_t MPU6050::calibrateGyro(int samples)
{
    if (samples <= 0)
        samples = 1;

    double sum = 0.0;
    int    ok  = 0;
    int16_t accel[3], gyro[3];

    for (int i = 0; i < samples; i++) {
        if (readRaw(accel, gyro) == ESP_OK) {
            sum += gyro[1]; // pitch axis
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (ok == 0) {
        ESP_LOGE(TAG, "calibrateGyro: no successful reads");
        return ESP_FAIL;
    }

    _gyro_offset_dps = (float)(sum / ok) / MPU6050_GYRO_LSB_PER_DPS;
    ESP_LOGI(TAG, "Gyro pitch offset: %.3f deg/s (%d/%d samples)", _gyro_offset_dps, ok, samples);
    return ESP_OK;
}

esp_err_t MPU6050::update(float dt_s)
{
    int16_t accel[3], gyro[3];
    esp_err_t err = readRaw(accel, gyro);
    if (err != ESP_OK)
        return err;

    float ax = (float)accel[0] / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)accel[2] / MPU6050_ACCEL_LSB_PER_G;
    float accel_pitch_deg = atan2f(-ax, az) * 180.0f / (float)M_PI;

    float gyro_rate_dps = (float)gyro[1] / MPU6050_GYRO_LSB_PER_DPS - _gyro_offset_dps;

    if (!_pitch_ready) {
        _pitch_deg   = accel_pitch_deg;
        _pitch_ready = true;
        return ESP_OK;
    }

    _pitch_deg = COMP_FILTER_ALPHA * (_pitch_deg + gyro_rate_dps * dt_s)
                 + (1.0f - COMP_FILTER_ALPHA) * accel_pitch_deg;

    return ESP_OK;
}
