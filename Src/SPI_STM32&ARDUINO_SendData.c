#include "stm32f411.h"
#include <string.h>

void delay(void){
		for(uint32_t i = 0; i<500000/2; i++);
}

void GPIO_ButtonInit(void){
		GPIO_Handle_t GPIOBtn;
		//this is the button GPIO configuration
		GPIOBtn.pGPIOx = GPIOA;
		GPIOBtn.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_0;
		GPIOBtn.GPIO_PinConfig->GPIO_PinMode = GPIO_MODE_IN;
		GPIOBtn.GPIO_PinConfig->GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig->GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GPIOBtn);
}
void SPI2_GPIOInits(void){
		GPIO_Handle_t SPIPins;
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig->GPIO_PinMode = GPIO_MODE_ALTFN;
		SPIPins.GPIO_PinConfig->GPIO_PinAltFunMode = 5;
		SPIPins.GPIO_PinConfig->GPIO_PinOPType = GPIO_OP_TYPE_PP;
		SPIPins.GPIO_PinConfig->GPIO_PinPuPdControl = GPIO_NO_PUPD;
		SPIPins.GPIO_PinConfig->GPIO_PinSpeed = GPIO_SPEED_FAST;

		// SCLK
		SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPins);

		// MOSI
		SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPins);

		// MISO but we don't need these for this question
		SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_14;
		GPIO_Init(&SPIPins);

		// NSS
		SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPins);
}
void SPI2_Inits(void){
		SPI_Handle_t SPI2handle;
		SPI2handle.pSPIx = SPI2;
		SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
		SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
		SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
		SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
		SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS pin

		SPI_Init(&SPI2handle);
}
int main(){
		char user_data[] = "Hello World";

		GPIO_ButtonInit();

		// This function is used to initialize the GPIO pins to behave as SPI2 Pins
		SPI2_GPIOInits();

		SPI2_Inits();

		// Setting SSOE Peripheral for enabling slave select output
		SPI_SSOEConfig(SPI2, ENABLE);

		// Implement the button functionality
		while(1){
				// Wait till the button is pressed
				while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

				// Avoid button de-bouncing related issues
				delay();

				// Enable the SPI2 Peripheral
				SPI_PeripheralControl(SPI2, ENABLE);

				// Send Data
				SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

				// Lets confirm that the SPI is not busy before disabling the peripheral
				while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
				// Disable the SPI2 Peripheral
				SPI_PeripheralControl(SPI2, DISABLE);
		}

		return 0;
}
