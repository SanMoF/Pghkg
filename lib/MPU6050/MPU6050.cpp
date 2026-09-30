#include "MPU6050.h"

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

// Complementary filter blend — trust the integrated gyro rate most of the
// time, correct slow drift with the accelerometer's absolute angle.
static const float COMP_FILTER_ALPHA = 0.98f;

MPU6050::MPU6050(i2c_port_num_t port, int sda_pin, int scl_pin,
                  uint32_t freq_hz, uint32_t timeout)
    : _port(port), _sda(sda_pin), _scl(scl_pin),
      _freq(freq_hz), _timeout_ms((int)timeout),
      _bus(nullptr), _dev(nullptr),
      _gyro_offset_dps(0.0f), _pitch_deg(0.0f), _pitch_ready(false)
{
}

MPU6050::~MPU6050()
{
    if (_dev) i2c_master_bus_rm_device(_dev);
    if (_bus) i2c_del_master_bus(_bus);
}

esp_err_t MPU6050::_read(uint8_t reg, uint8_t *data, size_t len) const
{
    // Single write-then-repeated-start-read transaction.
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, data, len, _timeout_ms);
    if (err == ESP_ERR_TIMEOUT)
        i2c_master_bus_reset(_bus); // release a stuck SDA/SCL so the next tick can recover
    return err;
}

esp_err_t MPU6050::_write(uint8_t reg, uint8_t value) const
{
    const uint8_t buf[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(_dev, buf, sizeof(buf), _timeout_ms);
    if (err == ESP_ERR_TIMEOUT)
        i2c_master_bus_reset(_bus);
    return err;
}

esp_err_t MPU6050::recover()
{
    if (_dev) { i2c_master_bus_rm_device(_dev); _dev = nullptr; }
    if (_bus) { i2c_del_master_bus(_bus);       _bus = nullptr; }
    _pitch_ready = false;
    return begin(false);
}

esp_err_t MPU6050::begin(bool scan_on_fail)
{
    if (_bus == nullptr) {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port          = _port;
        bus_cfg.sda_io_num        = (gpio_num_t)_sda;
        bus_cfg.scl_io_num        = (gpio_num_t)_scl;
        bus_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7; // filter spikes from a noisy jumper/ESC EMI
        bus_cfg.flags.enable_internal_pullup = true;

        esp_err_t berr = i2c_new_master_bus(&bus_cfg, &_bus);
        if (berr != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(berr));
            _bus = nullptr;
            return berr;
        }
    }

    if (_dev == nullptr) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = MPU6050_I2C_ADDR;
        dev_cfg.scl_speed_hz    = _freq;

        esp_err_t derr = i2c_master_bus_add_device(_bus, &dev_cfg, &_dev);
        if (derr != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(derr));
            _dev = nullptr;
            return derr;
        }
    }

    esp_err_t err;
    uint8_t whoami = 0;
    err = _read(MPU6050_REG_WHO_AM_I, &whoami, 1);
    // Print the raw result unconditionally — on a healthy bus this reads
    // 0x68 (MPU6050, or 0x69 with AD0 high). Register-compatible clones/successors
    // report 0x70 (MPU6500) or 0x71 (MPU9250) — the board in use returns 0x70.
    // Anything else (0x00, 0xFF, or an ESP_ERR_TIMEOUT/ESP_FAIL) means no ACK.
    printf("[MPU6050] WHO_AM_I read: err=%s value=0x%02X\n", esp_err_to_name(err), whoami);
    const bool idOk = (whoami & 0x7E) == 0x68 || whoami == 0x70 || whoami == 0x71;
    if (err != ESP_OK || !idOk) {
        ESP_LOGE(TAG, "MPU6050 not responding (WHO_AM_I=0x%02X) — check wiring/address", whoami);
        printf("[MPU6050] Scanning I2C%d bus for any responding device...\n", _port);
        if (scan_on_fail)
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
        if (i2c_master_probe(_bus, addr, 50) == ESP_OK) {
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
            sum += gyro[0]; // balance axis — see update()
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (ok == 0) {
        ESP_LOGE(TAG, "calibrateGyro: no successful reads");
        return ESP_FAIL;
    }

    _gyro_offset_dps = (float)(sum / ok) / MPU6050_GYRO_LSB_PER_DPS;
    ESP_LOGI(TAG, "Gyro balance-axis offset: %.3f deg/s (%d/%d samples)", _gyro_offset_dps, ok, samples);
    return ESP_OK;
}

esp_err_t MPU6050::update(float dt_s)
{
    int16_t accel[3], gyro[3];
    esp_err_t err = readRaw(accel, gyro);
    if (err != ESP_OK)
        return err;

    // Bench-confirmed with debug_mpu (hand-tilting the mounted board): ax/az
    // (classic "pitch") tracks LEFT/RIGHT tilt on this robot, not the
    // forward/back fall the wheel-balance loop needs to correct — the
    // sensor's mounting orientation puts the wheelbase's tilt axis on ay/az
    // instead. Sign (whether leaning forward reads positive or negative) is
    // NOT yet bench-verified — confirm with debug_mpu before arming the
    // ESCs, and flip the sign here if the PID pushes the wrong way.
    float ay = (float)accel[1] / MPU6050_ACCEL_LSB_PER_G;
    float az = (float)accel[2] / MPU6050_ACCEL_LSB_PER_G;
    float accel_pitch_deg = atan2f(-ay, az) * 180.0f / (float)M_PI;

    float gyro_rate_dps = (float)gyro[0] / MPU6050_GYRO_LSB_PER_DPS - _gyro_offset_dps;

    if (!_pitch_ready) {
        _pitch_deg   = accel_pitch_deg;
        _pitch_ready = true;
        return ESP_OK;
    }

    _pitch_deg = COMP_FILTER_ALPHA * (_pitch_deg + gyro_rate_dps * dt_s)
                 + (1.0f - COMP_FILTER_ALPHA) * accel_pitch_deg;

    return ESP_OK;
}
