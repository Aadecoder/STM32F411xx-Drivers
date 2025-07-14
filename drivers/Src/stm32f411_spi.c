#include "stm32f411.h"

/********************************************************
* @fn                - SPI_PeriClockControl

* @brief             - This function enables or disables peripheral clock for the given SPI port

* @param[in]         - base address of the SPI peripheral
* @param[in]         - ENABLE or DISABLE MACROS

* @return            -  none

* @Note              - none
********************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
		if(EnorDi == ENABLE){
				if(pSPIx == SPI1){ SPI1_PCLK_EN();}
				else if(pSPIx == SPI2){ SPI2_PCLK_EN();}
				else if(pSPIx == SPI3){ SPI3_PCLK_EN();}
				else if(pSPIx == SPI4){ SPI4_PCLK_EN();}
		}
		else{
				if(pSPIx == SPI1){ SPI1_PCLK_DI();}
				else if(pSPIx == SPI2){ SPI2_PCLK_DI();}
				else if(pSPIx == SPI3){ SPI3_PCLK_DI();}
				else if(pSPIx == SPI4){ SPI4_PCLK_DI();}
		}
}


/********************************************************
* @fn                - SPI_Init

* @brief             - This function initializes SPI pins

* @param[in]         - a pointer to the handle structure

* @return              -  none

* @Note                - none
********************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle){
	// Peripheral Clock Control
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure the SPI_CR1 Register
	uint32_t tempreg = 0;
	// 1. Configure the Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2. Configure the Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI Mode Should be cleared
		tempreg &= ~(1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDI Mode Should be set
		tempreg |= (1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI Mode Should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY must be set
		tempreg |= (1 << 10);
	}

	// 3. Configure the SCLK Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// 4. Configure the Data Frame Format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// 5. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	// 6. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// 7. Configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/********************************************************
* @fn                - SPI_DeInit

* @brief             - This function de-initializes SPI pins

* @param[in]         - base address of the SPI Port

* @return            -  none

* @Note              - none
********************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx){
			if(pSPIx == SPI1){ SPI1_REG_RESET();}
			else if(pSPIx == SPI2){ SPI2_REG_RESET();}
			else if(pSPIx == SPI3){ SPI3_REG_RESET();}
			else if(pSPIx == SPI4){ SPI4_REG_RESET();}
			else if(pSPIx == SPI5){ SPI5_REG_RESET();}
}

/********************************************************
* @fn                - SPI_GetFlagStatus

* @brief             - This function gets the flag status

* @param[in]         - base address of the SPI Port
* @param[in] 		 - Flag Name

* @return            - tells if flag is set or reset

* @Note              - none
********************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************
* @fn                - SPI_PeripheralControl

* @brief             - This function enables or disables the SPI Peripheral

* @param[in]         - base address of the SPI Port
* @param[in]         - ENABLE OR DISABLE

* @return            - None

* @Note              - None
********************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
		if (EnorDi == ENABLE){
				pSPIx->CR1 |= (1 << SPI_CR1_SPE);
		}else {
				pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
		}
}

/********************************************************
* @fn                - SPI_SSIConfig

* @brief             - This function configures the SPI SSI bit

* @param[in]         - base address of the SPI Port
* @param[in]         - ENABLE OR DISABLE

* @return            - None

* @Note              - None
********************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
		if (EnorDi == ENABLE){
				pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}else {
				pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/********************************************************
* @fn                - SPI_SSOEConfig

* @brief             - This function configures the SPI SSOE bit

* @param[in]         - base address of the SPI Port
* @param[in]         - ENABLE OR DISABLE

* @return            - None

* @Note              - None
********************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
		if (EnorDi == ENABLE){
				pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}else {
				pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}

/********************************************************
* @fn                - SPI_SendData

* @brief             - This function sends data

* @param[in]         - base address of the SPI Port
* @param[in]         - Transmission Buffer
* @param[in]         - Length

* @return            - None

* @Note              - Blocking API Function call will wait until all bytes are transmitted
                       also called Polling type as we are polling for the TXE flag to SET
********************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		// 1. Wait until the TxBuffer is empty (TXE is SET)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check DFF Bit in CR1 Register
		if((pSPIx->CR1) & (1 << SPI_CR1_DFF)){
			// 16 Bit Data Frame Format
			// Load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8 Bit Data Frame Format
			// Load the data into DR
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

/********************************************************
* @fn                - SPI_ReceiveData

* @brief             - This function recives data

* @param[in]         - base address of the SPI Port
* @param[in]         - Receive Buffer
* @param[in]         - Length

* @return            - None

* @Note              - Blocking API Function call will wait until all bytes are received
                       also called Polling type as we are polling for the RXE flag to SET
********************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		// 1. Wait until the RxBuffer is empty (RXNE is SET)
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check DFF Bit in CR1 Register
		if((pSPIx->CR1) & (1 << SPI_CR1_DFF)){
			// 16 Bit Data Frame Format
			// Load the data from the DR to RxBuffer Address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8 Bit Data Frame Format
			// Load the data from the DR to RxBuffer Address
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}



































