#ifndef _TCS34725_H
#define _TCS34725_H

#include "driver/i2c.h"
#include <stdint.h>

class TCS34725 {
public:
    TCS34725(); // constructor vacío

    // Inicializar el objeto sensor: pasar puerto I2C (I2C_NUM_0), y la dirección 7-bit (ej. 0x29)
    // NO configura el driver I2C: eso se hace en main con i2c_master_init()
    void begin(i2c_port_t i2c_num, uint8_t address);

    void setIntegrationTime(uint8_t atime);
    void setGain(uint8_t gain);

    // Lecturas
    void readRaw(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b);

private:
    i2c_port_t _i2c_num;
    uint8_t _address; // 7-bit I2C address
    static constexpr uint8_t COMMAND_BIT = 0x80;
    static constexpr uint8_t COMMAND_AUTO = 0xA0;

    void write8(uint8_t reg, uint8_t value);
    void readBytes(uint8_t reg, uint8_t *out, size_t len);
    void read16(uint8_t reg, uint16_t &value);
};

#endif