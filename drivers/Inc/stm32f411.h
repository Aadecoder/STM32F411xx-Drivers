/*
 * stm32f411.h
 *
 *  Created on: Jul 1, 2025
 *      Author: aditya
 */
#include <stdint.h>
#define __vo        volatile

#ifndef STM32F411_H_
#define STM32F411_H_

// Base Addresses for Flash and SRAM memory
#define FLASH_BASE              0x08000000U              
#define SRAM1_BASE              0x20000000U
#define ROM_BASE                0x1FFF0000U

// Base Addresses of Bus Domains
#define AHB1_BASE               0x40020000U
#define AHB2_BASE               0x50000000U
#define APB1_BASE               0x40000000U
#define APB2_BASE               0x40010000U
#define PERIH_BASE              0x40000000U

// Base Addresses of AHB1 Peripherals 
#define GPIOA_BASE              (AHB1_BASE + 0x0000)
#define GPIOB_BASE              (AHB1_BASE + 0x0400)
#define GPIOC_BASE              (AHB1_BASE + 0x0800)
#define GPIOD_BASE              (AHB1_BASE + 0x0C00)
#define GPIOE_BASE              (AHB1_BASE + 0x1000)
#define GPIOH_BASE              (AHB1_BASE + 0x1C00)
#define RCC_BASE                (AHB1_BASE + 0x3800)

// Base Addresses of APB1 Peripherals
#define SPI2_BASE               (APB1_BASE + 0x3800)
#define SPI3_BASE               (APB1_BASE + 0x3C00)
#define USART2_BASE             (APB1_BASE + 0x4400)
#define I2C1_BASE               (APB1_BASE + 0x5400)
#define I2C2_BASE               (APB1_BASE + 0x5800)
#define I2C3_BASE               (APB1_BASE + 0x5C00)

// Base Addresses of APB2 Peripherals
#define USART1_BASE             (APB2_BASE + 0x1000)
#define USART6_BASE             (APB2_BASE + 0x1400)
#define SPI1_BASE               (APB2_BASE + 0x3000)
#define SPI4_BASE               (APB2_BASE + 0x3400)
#define SPI5_BASE               (APB2_BASE + 0x5000)
#define SYSCFG_BASE             (APB2_BASE + 0x3800)
#define EXTI_BASE               (APB2_BASE + 0x3C00)

// GPIO Peripheral Registers Structure
typedef struct{
    __vo uint32_t MODER;        // Address Offset : 0x00
    __vo uint32_t OTYPER;       // Address Offset : 0x04
    __vo uint32_t OSPEEDR;      // Address Offset : 0x08
    __vo uint32_t PUPDR;        // Address Offset : 0x0C
    __vo uint32_t IDR;          // Address Offset : 0x10
    __vo uint32_t ODR;          // Address Offset : 0x14
    __vo uint32_t BSRR;         // Address Offset : 0x18
    __vo uint32_t LCKR;         // Address Offset : 0x1C
    __vo uint32_t AFR[2];       // Address Offset : 0x20 - 0x24
}GPIO_RegDef_t;

// Peripheral Defination Macros
#define GPIOA                   ((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB                   ((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC                   ((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD                   ((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE                   ((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOH                   ((GPIO_RegDef_t*)GPIOH_BASE)

// RCC Register Structure
typedef struct{
    __vo uint32_t CR;           // Address Offset : 0x00
    __vo uint32_t PLLCFGR;      // Address Offset : 0x04
    __vo uint32_t CFGR;         // Address Offset : 0x08
    __vo uint32_t CIR;          // Address Offset : 0x0C
    __vo uint32_t AHB1RSTR;     // Address Offset : 0x10
    __vo uint32_t AHB2RSTR;     // Address Offset : 0x14
    uint32_t RESERVED0;         // Address Offset : 0x18
    uint32_t RESERVED1;         // Address Offset : 0x1C
    __vo uint32_t APB1RSTR;     // Address Offset : 0x20
    __vo uint32_t APB2RSTR;     // Address Offset : 0x24
    uint32_t RESERVED2;         // Address Offset : 0x28
    uint32_t RESERVED3;         // Address Offset : 0x2C
    __vo uint32_t AHB1ENR;      // Address Offset : 0x30
    __vo uint32_t AHB2ENR;      // Address Offset : 0x34
    uint32_t RESERVED4;         // Address Offset : 0x38
    uint32_t RESERVED5;         // Address Offset : 0x3C
    __vo uint32_t APB1ENR;      // Address Offset : 0x40
    __vo uint32_t APB2ENR;      // Address Offset : 0x44
    uint32_t RESERVED6;         // Address Offset : 0x48
    uint32_t RESERVED7;         // Address Offset : 0x4C
    __vo uint32_t AHB1LPENR;    // Address Offset : 0x50
    __vo uint32_t AHB2LPENR;    // Address Offset : 0x54
    uint32_t RESERVED8;         // Address Offset : 0x58
    uint32_t RESERVED9;         // Address Offset : 0x5C
    __vo uint32_t APB1LPENR;    // Address Offset : 0x60
    __vo uint32_t APB2LPENR;    // Address Offset : 0x64
    uint32_t RESERVED10;        // Address Offset : 0x68
    uint32_t RESERVED11;        // Address Offset : 0x6C
    __vo uint32_t BDCR;         // Address Offset : 0x70
    __vo uint32_t CSR;          // Address Offset : 0x74
    uint32_t RESERVED12;        // Address Offset : 0x78
    uint32_t RESERVED13;        // Address Offset : 0x7C
    __vo uint32_t SSCGR;        // Address Offset : 0x80
    __vo uint32_t PLLI2SCFGR;   // Address Offset : 0x84
    uint32_t RESERVED14;        // Address Offset : 0x88
    __vo uint32_t DCKCFGR;      // Address Offset : 0x8C
}RCC_RegDef_t;

#define RCC                     ((RCC_RegDef_t*)RCC_BASE)

/*  Clock Enable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))

/* Clock Enable Macros for I2Cx Peripherals */
#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1 << 23))

/* Clock Enable Macros for SPIx Peripherals */
#define SPI1_PCLK_EN()     (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()     (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()     (RCC->APB2ENR |= (1 << 20))

/* Clock Enable Macros for USARTx Peripherals */
#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()   (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()   (RCC->APB2ENR |= (1 << 5))

/* Clock Enable Macros for SYSCFG Peripheral */
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR |= (1 << 14))

/*  Clock Disable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR |= (1 << 7))

/* Clock Disable Macros for I2Cx Peripherals */
#define I2C1_PCLK_DI()     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR |= (1 << 23))

/* Clock Disable Macros for SPIx Peripherals */
#define SPI1_PCLK_DI()     (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_DI()     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_DI()     (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_DI()     (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_DI()     (RCC->APB2ENR |= (1 << 20))

/* Clock Disable Macros for USARTx Peripherals */
#define USART1_PCLK_DI()   (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_DI()   (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_DI()   (RCC->APB2ENR |= (1 << 5))

/* Clock Disable Macros for SYSCFG Peripheral */
#define SYSCFG_PCLK_DI()     (RCC->APB2ENR |= (1 << 14))

// GPIO Clock Register Reset Macros
#define GPIOA_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)


// Some Macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET



#endif /* STM32F411_H_ */
