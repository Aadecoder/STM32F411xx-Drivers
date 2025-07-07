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

// ARM Cortex MX Processor NVIC ISERx Register Addresses
#define NVIC_ISER0               ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1               ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2               ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3               ((__vo uint32_t*)0xE000E10C)

// ARM Cortex MX Processor NVIC ICERx Register Addresses
#define NVIC_ICER0               ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1               ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2               ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3               ((__vo uint32_t*)0xE000E18C)

//ARM Cortex MX Processor Priority Register Address Calculation
#define NVIC_PR_BASE	         ((__vo uint32_t*)0xE000E400)

// ARM CORTEX MX PROCESSOR Number of Priority Bits implemented
#define NO_PR_BITS_IMPLEMENTED            4

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

// EXTI Peripheral Register Structure
typedef struct{
		__vo uint32_t IMR;          // Address Offset: 0x00
		__vo uint32_t EMR;          // Address Offset: 0x04
		__vo uint32_t RTSR;         // Address Offset: 0x08
		__vo uint32_t FTSR;         // Address Offset: 0x0C
		__vo uint32_t SWIER;        // Address Offset: 0x10
		__vo uint32_t PR;           // Address Offset: 0x14
}EXTI_RegDef_t;
#define EXTI  					((EXTI_RegDef_t*)EXTI_BASE)

// SYSCFG Peripheral Register Structure
typedef struct{
		__vo uint32_t MEMRMP;       // Address Offset: 0x00
		__vo uint32_t PMC;          // Address Offset: 0x04
		__vo uint32_t EXTICR[4];    // Address Offset: 0x08-0x14
		uint32_t RESERVED[2];       // Address Offset: 0x18-0x1C
	 __vo uint32_t CMPCR;           // Address Offset: 0x20
}SYSCFG_RegDef_t;
#define SYSCFG                  ((SYSCFG_RegDef_t*)SYSCFG_BASE)

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

// Return port code for given GPIOx base address required for configuring syscfg in interrupt
#define GPIO_BASEADDR_TO_CODE(x)     ( (x == GPIOA)?0:\
                                       (x == GPIOB)?1:\
                                       (x == GPIOC)?2:\
                                       (x == GPIOD)?3:\
                                       (x == GPIOE)?4:\
                                       (x == GPIOI)?7:0 )

// IRQ(Interrupt Request) Numbers of STM32F407x MCU
#define IRQ_NO_EXTI0             6
#define IRQ_NO_EXTI1             7
#define IRQ_NO_EXTI2             8
#define IRQ_NO_EXTI3             9
#define IRQ_NO_EXTI4             10
#define IRQ_NO_EXTI9_5           23
#define IRQ_NO_EXTI15_10         40

// Macros for NVIC IRQ Priority LEVELS
#define NVIC_IRQ_PRI0        0
#define NVIC_IRQ_PRI1        1
#define NVIC_IRQ_PRI2        2
#define NVIC_IRQ_PRI15       15


// Some Macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#endif /* STM32F411_H_ */
