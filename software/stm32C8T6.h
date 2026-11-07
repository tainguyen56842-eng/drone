#pragma once

#include <stdint.h>

struct STM32_RCC_TypeDef {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
};

struct STM32_GPIO_TypeDef {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
};

struct STM32_I2C_TypeDef {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
};

extern STM32_RCC_TypeDef* const RCC;
extern STM32_GPIO_TypeDef* const GPIOB;
extern STM32_I2C_TypeDef* const I2C1;

void STM32C8T6_ConfigureI2C1();
