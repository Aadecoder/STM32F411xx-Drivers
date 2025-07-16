#include "stm32f411.h"

// Prototypes of Handler/Helper functions being used to handle interrupts
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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

/********************************************************
* @fn                - SPI_IRQInterruptConfig

* @brief             - Enabling or Disabling IRQ's in NVIC registers

* @param[in]         - IRQ Number
* @param[in]         - Enable or disable

@return              - None 

@Note                - Anything done here is processor specific
*********************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	// Enabling or Disabling interrupt on a given IRQ line
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// Program ISER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
				
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// Program ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
				
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// Program ISER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64));
		}
	}else{
		if(IRQNumber <= 31){
			// Program ICER0 Register
			*NVIC_ICER0 &= ~( 1 << IRQNumber);
						
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// Program ICER1 Register
			*NVIC_ICER1 &= ~( 1 << (IRQNumber % 32));
						
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// Program ICER2 Register
			*NVIC_ICER2 &= ~( 1 << (IRQNumber % 64));
		}
	}	
}

/********************************************************
* @fn                - SPI_IRQPriorityConfig

* @brief             - Configure Priority Value for a given IRQ Number using Priotity Registers

* @param[in]         - IRQ Number
* @param[in]         - IRQ Priority

@return              - None 

@Note                - Anything done here is processor specific
*********************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	// Finding out the ipr register and section 
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// configure the priority register 
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/********************************************************
* @fn                - SPI_SendDataIT

* @brief             - This function sends data with Interrupt capability

* @param[in]         - SPI Handle Variable address
* @param[in]         - Transmission Buffer
* @param[in]         - Length

* @return            - The SPI State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		// Save the TxBuffer and len information in global varaibles
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// Mark the spi state as busy in the transmission
		// so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable the TXEIE bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

/********************************************************
* @fn                - SPI_ReceiveDataIT

* @brief             - This function receive data with Interrupt capability

* @param[in]         - SPI Handle Variable address
* @param[in]         - Reception Buffer
* @param[in]         - Length

* @return            - The SPI State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		// Save the RxBuffer and len information in global varaibles
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// Mark the spi state as busy in the reception
		// so that no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// Enable the RXNEIE bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/********************************************************
* @fn                - SPI_CloseTransmission

* @brief             - This function closes the transmission

* @param[in]         - SPI Handle Variable address

* @return            - None

* @Note              - None
********************************************************/

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/********************************************************
* @fn                - SPI_CloseReception

* @brief             - This function closes the reception

* @param[in]         - SPI Handle Variable address

* @return            - None

* @Note              - None
********************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/********************************************************
* @fn                - SPI_ClearOVRFlag

* @brief             - This function clears the OVR flag

* @param[in]         - SPI Register definations variable

* @return            - None

* @Note              - None
********************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/********************************************************
* @fn                - SPI_ApplicationEventCallback

* @brief             - This function calls the application callback after interrupt for handling

* @param[in]         - SPI Handle variable address

* @return            - None

* @Note              - None
********************************************************/
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	// this is a weak implementation 
	// the application may override this function
}

// Helper functions implementation
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit is 8 bit or 16 bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 Bit DFF
		// Load the data into DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		// 8 Bit DFF
		// Load the data into DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -- ;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen){
		// TxLen is zero so close the transmission and inform the application that the tx is over
		// this prevents the interrupts from setting up the txe flag
		SPI_CloseTransmission(pSPIHandle);
		// Inform the application that the tx is over
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit is 8 bit or 16 bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16 Bit DFF
		// Recieve the data from the DR
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{
		// 8 Bit DFF
		// Receive the data from DR
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -- ;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	// clear the OVR bit by read access to the DR and SR register
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	(void)temp;
	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/********************************************************
* @fn                - SPI_IRQHandling

* @brief             -  Handles the Interrupts 

* @param[in]         - IRQ Number

@return              - None 

@Note                - Anything done here is processor specific
*********************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1, temp2;
	// Check for TXE
	temp1 = pSPIHandle->pSPIx->SR |= (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}
	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SR |= (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		// Handle RXNE
		spi_rxe_interrupt_handle(pSPIHandle);
	}
	// check for OVR error
	temp1 = pSPIHandle->pSPIx->SR |= (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		// Handle OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}




