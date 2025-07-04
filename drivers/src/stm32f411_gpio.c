/*
 * stm32f411_gpio.c
 *
 *  Created on: Jul 5, 2025
 *      Author: aditya
 */
#include "stm32f411_gpio.h"

// GPIO API Implementations

/********************************************************
* @fn                - GPIO_PeriClockControl

* @brief             - This function enables or disables peripheral clock for the given GPIO port

* @param[in]         - base address of the GPIO peripheral
* @param[in]         - ENABLE or DISABLE MACROS

* @return            -  none

* @Note              - none
********************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if(pGPIOx == GPIOA){GPIOA_PCLK_EN();}
		else if(pGPIOx == GPIOB){GPIOB_PCLK_EN();}
		else if(pGPIOx == GPIOC){GPIOC_PCLK_EN();}
		else if(pGPIOx == GPIOD){GPIOD_PCLK_EN();}
		else if(pGPIOx == GPIOE){GPIOE_PCLK_EN();}
		else if(pGPIOx == GPIOH){GPIOH_PCLK_EN();}
	}else {
		if(pGPIOx == GPIOA){GPIOA_PCLK_DI();}
		else if(pGPIOx == GPIOB){GPIOB_PCLK_DI();}
		else if(pGPIOx == GPIOC){GPIOC_PCLK_DI();}
		else if(pGPIOx == GPIOD){GPIOD_PCLK_DI();}
		else if(pGPIOx == GPIOE){GPIOE_PCLK_DI();}
		else if(pGPIOx == GPIOH){GPIOH_PCLK_DI();}
	}
}































