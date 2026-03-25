#include "TCS34725.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stdio.h>
#include "driver/i2c.h"

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 100000 // 100 kHz standard speed

TCS34725::TCS34725() : _i2c_num(I2C_NUM_0), _address(0x29) {}

void TCS34725::begin(i2c_port_t i2c_num, uint8_t address)
{
    _i2c_num = i2c_num;
    _address = address;

    // Configure I2C if not already done
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ_HZ,
        },
        .clk_flags = 0,
    };
    i2c_param_config(_i2c_num, &conf);
    i2c_driver_install(_i2c_num, I2C_MODE_MASTER, 0, 0, 0);

    // Power on (PON), wait, then enable ADC (AEN)
    write8(0x00, 0x01); // ENABLE = PON
    vTaskDelay(pdMS_TO_TICKS(3));
    write8(0x00, 0x03); // ENABLE = PON + AEN
    vTaskDelay(pdMS_TO_TICKS(700)); // allow initial integration
}


void TCS34725::setIntegrationTime(uint8_t atime)
{
    write8(0x01, atime);
}

void TCS34725::setGain(uint8_t gain)
{
    write8(0x0F, gain);
}

void TCS34725::readRaw(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b)
{
    // Read clear, red, green, blue (each 16-bit little-endian)
    read16(0x14, c);
    read16(0x16, r);
    read16(0x18, g);
    read16(0x1A, b);
}

void TCS34725::write8(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = COMMAND_BIT | reg;
    data[1] = value;
    esp_err_t err = i2c_master_write_to_device(_i2c_num, _address, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        printf("i2c write8 err %s (reg 0x%02X)\n", esp_err_to_name(err), reg);
    }
}

void TCS34725::readBytes(uint8_t reg, uint8_t *out, size_t len)
{
    // Use write-then-read convenience function from esp-idf
    uint8_t cmd = (uint8_t)(COMMAND_AUTO | reg); // use auto-increment when reading multiple bytes
    esp_err_t err = i2c_master_write_read_device(_i2c_num, _address, &cmd, 1, out, len, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        printf("i2c readBytes err %s (reg 0x%02X len %d)\n", esp_err_to_name(err), reg, (int)len);
        // zero the output on error
        for (size_t i = 0; i < len; ++i) out[i] = 0;
    }
}

void TCS34725::read16(uint8_t reg, uint16_t &value)
{
    uint8_t buf[2] = {0, 0};
    // read two bytes starting at reg: low byte then high byte
    readBytes(reg, buf, 2);
    value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}