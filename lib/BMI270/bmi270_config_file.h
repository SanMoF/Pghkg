#pragma once

#include <stdint.h>
#include <stddef.h>

// Bosch's mandatory BMI270 initialization blob ("config file" / firmware).
// The sensor stays non-operational (INTERNAL_STATUS stuck at 0x00, no valid
// accel/gyro data) until this exact 8192-byte array is burst-written to it
// during begin(). Populated verbatim from Bosch Sensortec's BMI270 Sensor
// API repository (github.com/boschsensortec/BMI270_SensorAPI, bmi270.c,
// array `bmi270_config_file[]`) — see bmi270_config_file.cpp.
//
// If this is ever cleared back to length 0, BMI270::begin() detects it and
// refuses to proceed with a clear printf error rather than silently running
// against an uninitialized sensor.
extern const uint8_t bmi270_config_file[];
extern const size_t  bmi270_config_file_len;
