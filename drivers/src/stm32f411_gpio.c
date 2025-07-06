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

/********************************************************
* @fn                - GPIO_Init

* @brief             - This function initializes GPIO pins

* @param[in]         - a pointer to the handle structure

* @return              -  none

* @Note                - none
********************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint8_t temp=0;

	// 1. Configure the mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode <= GPIO_MODE_ANALOG){
		// Non Interrupt
		temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	temp = 0;
	// 2. Configure the Output Type
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0X1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	// 3. Configure the Output Speed
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// 4. Configure the Pull up / Pull Down Resistors
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	// 5. Configure the Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber % 4;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig->GPIO_PinAltFunMode << (4 * temp2));
	}
}

/********************************************************
* @fn                - GPIO_DeInit

* @brief             - This function de-initializes GPIO pins

* @param[in]         - base address of the GPIO Port

* @return            -  none

* @Note              - none
********************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){GPIOA_REG_RESET();}
	else if(pGPIOx == GPIOB){GPIOB_REG_RESET();}
	else if(pGPIOx == GPIOC){GPIOC_REG_RESET();}
	else if(pGPIOx == GPIOD){GPIOD_REG_RESET();}
	else if(pGPIOx == GPIOE){GPIOE_REG_RESET();}
	else if(pGPIOx == GPIOH){GPIOH_REG_RESET();}
}

/********************************************************
* @fn                - GPIO_ReadFromInputPin

* @brief             - This function reads from GPIO input pin

* @param[in]         - base address of the GPIO Port
* @param[in]         - Pin number

* @return            - either 0 or 1

* @Note              - Read the corresponding bit position in the input data register
********************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> (PinNumber) & 0x00000001);
	return value;
}

/********************************************************
* @fn                - GPIO_ReadFromInputPort

* @brief             - This function reads from GPIO input port

* @param[in]         - base address of the GPIO Port

* @return            - 16 bit unsigned integer

* @Note              - Read all the bit positions in the input data register
********************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/********************************************************
* @fn                - GPIO_WriteToOutputPin

* @brief             - This function writes to the Output data register

* @param[in]         - base address of the GPIO Port
* @param[in]         - Pin Number
* @param[in]         - Value to output(GPIO_PIN_SET/GPIO_PIN_RESET)

* @return            - none

* @Note              - writes a 0 or 1 value to the output data register at the bit field corresponding to the pin numeber
********************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/********************************************************
* @fn                - GPIO_WriteToOutputPort

* @brief             - This function writes to the output data register

* @param[in]         - base address of the GPIO Port
* @param[in]         - Value to output

* @return            - none

* @Note              - writes a value to the output data register
********************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR |= Value;
}

/********************************************************
* @fn                - GPIO_ToggleOutputPin

* @brief             - This function toggles output pin

* @param[in]         - base address of the GPIO Port
* @param[in]         - Pin Number

* @return            - none

* @Note              -
********************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}


























