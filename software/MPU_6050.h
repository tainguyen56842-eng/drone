#pragma once

#include <stdint.h>

struct MPU6050Config {
    uint8_t device_address;
    uint8_t who_am_i_register;
    uint8_t expected_device_id;
    uint8_t power_management_register;
    uint8_t power_management_value;
    uint8_t sample_rate_register;
    uint8_t sample_rate_value;
    uint8_t config_register;
    uint8_t config_value;
    uint8_t gyro_config_register;
    uint8_t gyro_config_value;
    uint8_t accel_config_register;
    uint8_t accel_config_value;
    uint8_t output_register;
};

struct MPU6050Reading {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
};

extern const MPU6050Config MPU6050_DEFAULT_CONFIG;
