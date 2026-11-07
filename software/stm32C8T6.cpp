#include "stm32C8T6.h"

#include <stddef.h>

static_assert(offsetof(STM32_RCC_TypeDef, APB2ENR) == 0x18, "RCC register offset");
static_assert(offsetof(STM32_RCC_TypeDef, APB1ENR) == 0x1C, "RCC register offset");
static_assert(offsetof(STM32_GPIO_TypeDef, IDR) == 0x08, "GPIO register offset");
static_assert(offsetof(STM32_I2C_TypeDef, DR) == 0x10, "I2C register offset");
static_assert(offsetof(STM32_I2C_TypeDef, SR1) == 0x14, "I2C register offset");
static_assert(offsetof(STM32_I2C_TypeDef, SR2) == 0x18, "I2C register offset");
static_assert(offsetof(STM32_I2C_TypeDef, CCR) == 0x1C, "I2C register offset");

namespace {
constexpr uintptr_t kRccBase = 0x40021000;
constexpr uintptr_t kGpioBBase = 0x40010C00;
constexpr uintptr_t kI2c1Base = 0x40005400;
constexpr uint32_t kApb1ClockHz = 36000000;
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint32_t kRccGpioBEnable = 1u << 3;
constexpr uint32_t kRccI2c1Enable = 1u << 21;
constexpr uint32_t kI2cPeripheralEnable = 1u << 0;
}

STM32_RCC_TypeDef* const RCC =
    reinterpret_cast<STM32_RCC_TypeDef*>(kRccBase);
STM32_GPIO_TypeDef* const GPIOB =
    reinterpret_cast<STM32_GPIO_TypeDef*>(kGpioBBase);
STM32_I2C_TypeDef* const I2C1 =
    reinterpret_cast<STM32_I2C_TypeDef*>(kI2c1Base);

void STM32C8T6_ConfigureI2C1() {
    RCC->APB2ENR |= kRccGpioBEnable;
    RCC->APB1ENR |= kRccI2c1Enable;

    GPIOB->BSRR = (1u << 6) | (1u << 7);
    GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0xFFu << 24);

    RCC->APB1RSTR |= kRccI2c1Enable;
    RCC->APB1RSTR &= ~kRccI2c1Enable;

    I2C1->CR1 = 0;
    I2C1->CR2 = (I2C1->CR2 & ~0x3Fu) | (kApb1ClockHz / 1000000);
    I2C1->CCR = kApb1ClockHz / (2 * kI2cClockHz);
    I2C1->TRISE = (kApb1ClockHz / 1000000) + 1;
    I2C1->CR1 = kI2cPeripheralEnable;
}
