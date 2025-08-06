# STM32F411xx Drivers

**Bare-metal driver implementations for STM32F411xx series microcontrollers**

---

## 🧩 Overview

This repository provides low-level, polling-based peripheral drivers for the STM32F411xx MCU, written from scratch. Ideal for learning register-level programming and working with peripherals like GPIO.

---

## ⚙️ Peripherals

- ✅ GPIO
- ✅ SPI
- ✅ I2C
- ✅ USART

---

## Project Structure

```bash
STM32F411CEU6_Drivers/
├── .settings/                     ← IDE configuration files
├── .Debug/                         
├── drivers/                       ← Custom peripheral drivers
│   ├── Inc/                       ← Driver headers
│   │    └── stm32f411.h           ← MCU specific header file
│   │    └── stm32f411_gpio.h      ← GPIO header file
│   │    └── stm32f411_spi.h       ← SPI header file
│   │    └── stm32f411_i2c.h       ← I2C header file
│   │    └── stm32f411_usart.h     ← USART header file
│   │    └── stm32f411_rcc.h       ← RCC header file
│   └── Src/
│        └── stm32f411_gpio.c      ← GPIO API Implementation source file
│        └── stm32f411_spi.c       ← SPI API Implementation source file
│        └── stm32f411_i2c.c       ← I2C API Implementation source file
│        └── stm32f411c_usart.c    ← USART API Implementation source file
│        └── stm32f411_rcc.c       ← RCC API Implementation source file
├── Inc/                           ← Application-level headers
├── Src/                           ← Application source code
├── Startup/                       ← Startup assembly and system files
├── STM32F411CEUX_FLASH.ld         ← Linker script for flash memory
├── STM32F411CEUX_RAM.ld           ← Linker script for RAM (if used)
├── .cproject                      
├── .project                       
└── README.md                      ← Project documentation
```

---

## Contact

- Author : Aditya Rajput
- Email : adityandr8274@gmail.com