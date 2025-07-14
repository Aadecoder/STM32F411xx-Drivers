#include "stm32f411.h"
#include <string.h>

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig->GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig->GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig->GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig->GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig->GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig->GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
}

void SPI2Inits(void){
	SPI_Handle_t SPI2HANDLE;
	SPI2HANDLE.pSPIx = SPI2;
	SPI2HANDLE.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2HANDLE.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2HANDLE.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2HANDLE.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2HANDLE.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2HANDLE.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI2HANDLE.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	SPI_Init(&SPI2HANDLE);
}

int main(void){
	char userData[] = "Hello World";

	// Initializing GPIO Pins to behave as SPI2 Pin
	SPI2_GPIOInits();

	// SPI2 Initialization
	SPI2Inits();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	// First Send length information
	uint8_t dataLen = strlen(userData);
	SPI_SendData(SPI2, &dataLen, 1);

	// Send Data
	SPI_SendData(SPI2, (uint8_t*)userData, strlen(userData));

	// Confirm that the SPI is not busy before disabling
	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
	// Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}
