#include "MPU_6050.h"
#include "stm32C8T6.h"

#include <math.h>
#include <stdint.h>

namespace {
constexpr uint32_t kI2cTimeout = 100000;
constexpr uint32_t kI2cStart = 1u << 8;
constexpr uint32_t kI2cStop = 1u << 9;
constexpr uint32_t kI2cAck = 1u << 10;
constexpr uint32_t kI2cPos = 1u << 11;
constexpr uint32_t kI2cSb = 1u << 0;
constexpr uint32_t kI2cAddr = 1u << 1;
constexpr uint32_t kI2cBtf = 1u << 2;
constexpr uint32_t kI2cRxne = 1u << 6;
constexpr uint32_t kI2cTxe = 1u << 7;
constexpr uint32_t kI2cBusy = 1u << 1;
constexpr uint32_t kI2cErrorFlags =
    (1u << 8) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 14);
constexpr float kRadiansToDegrees = 57.2957795f;

bool waitForFlag(uint32_t flag) {
    for (uint32_t timeout = 0; timeout < kI2cTimeout; ++timeout) {
        const uint32_t status = I2C1->SR1;
        if ((status & kI2cErrorFlags) != 0) {
            return false;
        }
        if ((status & flag) != 0) {
            return true;
        }
    }
    return false;
}

bool waitForBusIdle() {
    for (uint32_t timeout = 0; timeout < kI2cTimeout; ++timeout) {
        if ((I2C1->SR2 & kI2cBusy) == 0) {
            return true;
        }
    }
    return false;
}

void clearAddressFlag() {
    const volatile uint32_t status1 = I2C1->SR1;
    const volatile uint32_t status2 = I2C1->SR2;
    (void)status1;
    (void)status2;
}

void stopTransaction() {
    I2C1->CR1 |= kI2cStop | kI2cAck;
    I2C1->CR1 &= ~kI2cPos;
    I2C1->SR1 = ~kI2cErrorFlags;
}

bool startWriteAddress(uint8_t device_address) {
    if (!waitForBusIdle()) {
        return false;
    }
    I2C1->CR1 |= kI2cStart;
    if (!waitForFlag(kI2cSb)) {
        return false;
    }
    I2C1->DR = static_cast<uint32_t>(device_address) << 1;
    if (!waitForFlag(kI2cAddr)) {
        return false;
    }
    clearAddressFlag();
    return true;
}

bool writeRegister(uint8_t device_address, uint8_t reg, uint8_t value) {
    if (!startWriteAddress(device_address)) {
        stopTransaction();
        return false;
    }

    if (!waitForFlag(kI2cTxe)) {
        stopTransaction();
        return false;
    }
    I2C1->DR = reg;

    if (!waitForFlag(kI2cBtf)) {
        stopTransaction();
        return false;
    }
    I2C1->DR = value;

    if (!waitForFlag(kI2cBtf)) {
        stopTransaction();
        return false;
    }
    I2C1->CR1 |= kI2cStop;
    return true;
}

bool readRegisters(
    uint8_t device_address,
    uint8_t first_register,
    uint8_t* data,
    uint8_t length) {
    if (data == nullptr || length == 0 ||
        !startWriteAddress(device_address)) {
        stopTransaction();
        return false;
    }

    if (!waitForFlag(kI2cTxe)) {
        stopTransaction();
        return false;
    }
    I2C1->DR = first_register;
    if (!waitForFlag(kI2cBtf)) {
        stopTransaction();
        return false;
    }

    I2C1->CR1 |= kI2cStart;
    if (!waitForFlag(kI2cSb)) {
        stopTransaction();
        return false;
    }
    I2C1->DR = (static_cast<uint32_t>(device_address) << 1) | 1u;
    if (!waitForFlag(kI2cAddr)) {
        stopTransaction();
        return false;
    }

    if (length == 1) {
        I2C1->CR1 &= ~kI2cAck;
        clearAddressFlag();
        I2C1->CR1 |= kI2cStop;
        if (!waitForFlag(kI2cRxne)) {
            stopTransaction();
            return false;
        }
        data[0] = static_cast<uint8_t>(I2C1->DR);
        I2C1->CR1 |= kI2cAck;
        return true;
    }

    if (length == 2) {
        I2C1->CR1 |= kI2cPos;
        I2C1->CR1 &= ~kI2cAck;
        clearAddressFlag();
        if (!waitForFlag(kI2cBtf)) {
            stopTransaction();
            return false;
        }
        I2C1->CR1 |= kI2cStop;
        data[0] = static_cast<uint8_t>(I2C1->DR);
        data[1] = static_cast<uint8_t>(I2C1->DR);
        I2C1->CR1 |= kI2cAck;
        I2C1->CR1 &= ~kI2cPos;
        return true;
    }

    clearAddressFlag();
    uint8_t index = 0;
    while (length - index > 3) {
        if (!waitForFlag(kI2cRxne)) {
            stopTransaction();
            return false;
        }
        data[index++] = static_cast<uint8_t>(I2C1->DR);
    }

    if (!waitForFlag(kI2cBtf)) {
        stopTransaction();
        return false;
    }
    I2C1->CR1 &= ~kI2cAck;
    data[index++] = static_cast<uint8_t>(I2C1->DR);

    if (!waitForFlag(kI2cBtf)) {
        stopTransaction();
        return false;
    }
    I2C1->CR1 |= kI2cStop;
    data[index++] = static_cast<uint8_t>(I2C1->DR);
    data[index] = static_cast<uint8_t>(I2C1->DR);
    I2C1->CR1 |= kI2cAck;
    return true;
}

int16_t toSigned(uint8_t high, uint8_t low) {
    const uint16_t value =
        (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    return static_cast<int16_t>(value);
}

bool initializeMpu6050(const MPU6050Config& config) {
    uint8_t device_id = 0;
    if (!readRegisters(
            config.device_address,
            config.who_am_i_register,
            &device_id,
            1) ||
        device_id != config.expected_device_id) {
        return false;
    }

    return writeRegister(
               config.device_address,
               config.power_management_register,
               config.power_management_value) &&
           writeRegister(
               config.device_address,
               config.sample_rate_register,
               config.sample_rate_value) &&
           writeRegister(
               config.device_address,
               config.config_register,
               config.config_value) &&
           writeRegister(
               config.device_address,
               config.gyro_config_register,
               config.gyro_config_value) &&
           writeRegister(
               config.device_address,
               config.accel_config_register,
               config.accel_config_value);
}

bool readMpu6050(
    const MPU6050Config& config,
    MPU6050Reading& reading) {
    uint8_t data[14] = {};
    if (!readRegisters(
            config.device_address,
            config.output_register,
            data,
            sizeof(data))) {
        return false;
    }

    reading.accel_x_g = toSigned(data[0], data[1]) / 16384.0f;
    reading.accel_y_g = toSigned(data[2], data[3]) / 16384.0f;
    reading.accel_z_g = toSigned(data[4], data[5]) / 16384.0f;
    reading.temperature_c = toSigned(data[6], data[7]) / 340.0f + 36.53f;
    reading.gyro_x_dps = toSigned(data[8], data[9]) / 131.0f;
    reading.gyro_y_dps = toSigned(data[10], data[11]) / 131.0f;
    reading.gyro_z_dps = toSigned(data[12], data[13]) / 131.0f;
    return true;
}
}

volatile float roll_degrees = 0.0f;
volatile float pitch_degrees = 0.0f;
volatile float temperature_celsius = 0.0f;
volatile float gyro_x_degrees_per_second = 0.0f;
volatile float gyro_y_degrees_per_second = 0.0f;
volatile float gyro_z_degrees_per_second = 0.0f;
volatile uint8_t mpu6050_ready = 0;
volatile uint8_t mpu6050_sample_valid = 0;

int main() {
    STM32C8T6_ConfigureI2C1();
    if (!initializeMpu6050(MPU6050_DEFAULT_CONFIG)) {
        mpu6050_ready = 0;
        while (true) {
        }
    }
    mpu6050_ready = 1;

    MPU6050Reading reading = {};
    while (true) {
        if (!readMpu6050(MPU6050_DEFAULT_CONFIG, reading)) {
            mpu6050_sample_valid = 0;
            continue;
        }

        roll_degrees =
            atan2f(reading.accel_y_g, reading.accel_z_g) *
            kRadiansToDegrees;
        pitch_degrees =
            atan2f(
                -reading.accel_x_g,
                sqrtf(
                    reading.accel_y_g * reading.accel_y_g +
                    reading.accel_z_g * reading.accel_z_g)) *
            kRadiansToDegrees;
        temperature_celsius = reading.temperature_c;
        gyro_x_degrees_per_second = reading.gyro_x_dps;
        gyro_y_degrees_per_second = reading.gyro_y_dps;
        gyro_z_degrees_per_second = reading.gyro_z_dps;
        mpu6050_sample_valid = 1;
    }
}
