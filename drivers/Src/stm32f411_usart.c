#include "stm32f411.h"

/********************************************************
* @fn                - USART_PeriClockControl

* @brief             - This function enables or disables peripheral clock for the given USART port

* @param[in]         - base address of the USART peripheral
* @param[in]         - ENABLE or DISABLE macros

* @return              -  none

* @Note                - none
********************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
		if(EnorDi == ENABLE){
				if(pUSARTx == USART1){ USART1_PCLK_EN();}
				else if(pUSARTx == USART2){ USART2_PCLK_EN();}
				else if(pUSARTx == USART6){ USART6_PCLK_EN();}
		}
		else{
				if(pUSARTx == USART1){ USART1_PCLK_DI();}
				else if(pUSARTx == USART2){ USART2_PCLK_DI();}
				else if(pUSARTx == USART6){ USART6_PCLK_DI();}
		}
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - This function sets the baud rate of the USART peripheral
 *
 * @param[in]         - USART port address
 * @param[in]         - Baud rate value
 *
 * @return            - None
 *
 * @Note              - None

 **********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){
	// Variable to hold the APB clock
	uint32_t PCLKx;
	uint32_t usartdiv;

	// Variable to hold the Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	// Get the value of the APB Bus Clock in the varaible PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6){
		// USART1 and USART6 are hanging on APB2 Bus
		PCLKx = RCC_GetPCLK2Value();
	}else{
		// USART2 is hanging on APB1 Bus
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for the OVER8 Configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
		// OVER8 = 1; Over Sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}else{
		// OVER8 = 0; Over Sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate the Mantissa Part
	M_part = usartdiv / 100;

	// Place the Mantissa part in the appropriate bit position
	tempreg |= M_part << 4;

	// Calculate the Fraction Part
	F_part = (usartdiv - (M_part * 100));

	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8)){
		// Over Sampling is 8
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
	}else{
		// Over Sampling is 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
	}

	// Place the fraction part in the appropriate bit position
	tempreg |= F_part;

	// Finally put the value in the BRR register
	pUSARTx->BRR = tempreg;
}

/********************************************************
* @fn                - USART_Init

* @brief             - This function initializes USART pins

* @param[in]         - a pointer to the handle structure

* @return            -  none

* @Note              - none
********************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t tempreg=0;

	// Configuration of CR1

	// Enable the clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable the USART Tx and Rx engines
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		tempreg |= (1 << USART_CR1_RE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		tempreg |= (1 << USART_CR1_TE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

	// Configure the Word Length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// Configure the Parity bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		// Enable the Parity control bit
		tempreg |= ( 1 << USART_CR1_PCE);
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		// Enable the Parity Control bit
		tempreg |= ( 1 << USART_CR1_PCE);

		// Enable the ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	// Configuration of CR2
	tempreg = 0;

	// Configure the number of stop bits
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << (USART_CR2_STOP));

	pUSARTHandle->pUSARTx->CR2 = tempreg;

	// Configuration of CR3
	tempreg = 0;

	// Configure USART Hardware Flow Control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		// Enable the CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		// Enable the RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		// Enable both CTS and RTS
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	// Configuration of BRR (Baud Rate Register)
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/********************************************************
* @fn 		- USART_DeInit

* @brief 	- This function de-initializes USART pins

* @param[in] - base address of the USART Port

* @return 	- none

* @Note 	- none
********************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx){
	if(pUSARTx == USART1){ USART1_REG_RESET();}
	else if(pUSARTx == USART2){ USART2_REG_RESET();}
	else if(pUSARTx == USART6){ USART6_REG_RESET();}
}

/********************************************************
* @fn                - USART_PeripheralControl

* @brief             - This function enables or disables the USART Peripheral 

* @param[in]         - base address of the USART Port
* @param[in]         - ENABLE OR DISABLE

* @return            - None

* @Note              - None
********************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
		if (EnorDi == ENABLE){
				pUSARTx->CR1 |= (1 << USART_CR1_UE);
		}else {
				pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
		}
}

/********************************************************
* @fn                - USART_GetFlagStatus

* @brief             - This function returns the status of flags

* @param[in]         - base address of the Port
* @param[in]         - Flag Name

* @return            - uint8_t flag set or reset

* @Note              - Helper function
********************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName){
		if (pUSARTx->SR & FlagName){
				return FLAG_SET;
		}
		return FLAG_RESET;
}

/********************************************************
* @fn                - USART_ClearFlag

* @brief             - This function clears the flag

* @param[in]         - base address of the Port
* @param[in]         - Flag Name

* @return            - None

* @Note              - Helper function
********************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName);
}

/********************************************************
* @fn                - USART_SendData

* @brief             - This function sends data

* @param[in]         - base address of the handle
* @param[in]         - Transmission Buffer
* @param[in]         - Length

* @return            - None

* @Note              - None
********************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;

	// Loop over until Len number of bytes are transferred
	for(uint32_t i=0;i<Len;i++){
		// Wait until the TXE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TXE));

		// Check for the Word Length
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			// Load the data with 2 bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// Check for the Parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// No parity used in this transfer
				// 9 bits of user data will be send
				// Increment the pTxBuffer twice
				pTxBuffer += 2;
			}else{
				// Parity is used
				// 8 bits of user data will be send
				pTxBuffer++;
			}
		}else{
			// For 8 bits of data transfer
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0xFF);
			pTxBuffer++;
		}
	}
	// wait till the TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_TC));
}

/********************************************************
* @fn                - USART_ReceiveData

* @brief             - This function receives data

* @param[in]         - Handle variable
* @param[in]         - Reception Buffer
* @param[in]         - Length

* @return            - None

* @Note              - None
********************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	for(uint32_t i=0;i<Len;i++){
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_SR_RXNE));

		// Check for the word length
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			// We are going to receive 9 bits of data frame
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// no parity is used therefore 9 bits of user data will be received
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer += 2;
			}else{
				// Parity is used then 8 bits of user data will be received
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}else{
			// We are going to receive 8 bits of data frame
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}else{
				// parity is used so 7 bits of user data will be received
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			// increment the buffer address
			pRxBuffer++;
		}
	}
}

/********************************************************
* @fn                - USART_SendDataIT

* @brief             - This function sends data with Interrupt capability

* @param[in]         - USART Handle Variable address
* @param[in]         - Transmission Buffer
* @param[in]         - Length

* @return            - The TX State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;
	if(txstate != USART_BUSY_IN_TX){
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable the interrupt for TXE
		pUSARTHandle->pUSARTx->SR |= (1 << USART_SR_TXE);

		// Enable the interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCEIE);
	}
	return txstate;
}

/********************************************************
* @fn                - USART_ReceiveDataIT

* @brief             - This function receives data with Interrupt capability

* @param[in]         - USART Handle Variable address
* @param[in]         - Rx Buffer
* @param[in]         - Length

* @return            - The USART State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->RxBusyState;
	if(rxstate != USART_BUSY_IN_RX){
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->pRxBuffer = pRxBuffer;

		// Enable the interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

/********************************************************
* @fn                - USART_IRQInterruptConfig

* @brief             - Enabling or Disabling IRQ's in NVIC registers

* @param[in]         - IRQ Number
* @param[in]         - Enable or disable

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 ){
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 ){
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else{
		if(IRQNumber <= 31){
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 ){
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 ){
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/********************************************************
* @fn                - USART_IRQPriorityConfig

* @brief             - Configure Priority Value for a given IRQ Number using Priority Registers

* @param[in]         - IRQ Number
* @param[in]         - IRQ Priority

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE + iprx ) |=  ( IRQPriority << shift_amount );

}

/********************************************************
* @fn                - USART_IRQHandling

* @brief             - Handles the interrupts triggered

* @param[in]         - USART Handle variables

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle){
	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;

	// Check for the TC flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCEIE);

	if(temp1 && temp2){
		// Close the Transmission and call the application callback if TxLen is zero
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			// check the TxLen and if it is zero then close the transmission
			if(!pUSARTHandle->TxLen){
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCEIE);

				pUSARTHandle->TxBusyState = USART_READY;
				pUSARTHandle->pTxBuffer = NULL;
				pUSARTHandle->TxLen = 0;

				// call the application callback
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	// Check for the TXE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2){
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){
			// Keep sending data until TxLen becomes zero
			if(pUSARTHandle->TxLen > 0){
				// Check for the word length
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					// Load the DR with 2 bytes
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// check for the parity bit
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						// No parity is used in transfer so 9 bits of data will be send
						pUSARTHandle->pTxBuffer +=2;
						pUSARTHandle->TxLen -= 2;
					}else{
						// 8 bits of data will be send
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}else{
					// 8 Bits data frame
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}
			}
			if(pUSARTHandle->TxLen == 0){
				// Clear the TXEIE bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	// Check for RXNE flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){
			if(pUSARTHandle->RxLen > 0){
				// Check the word length
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					// Check for Parity
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						// No Parity is used so 9 bits of user data will be received
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
						pUSARTHandle->pRxBuffer += 2;
						pUSARTHandle->RxLen -= 2;
					}else{
						// Parity is enabled so 8 bits of data will be received
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
				}else{
					// 8 bits of data frame
					// check for parity bits
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						// read 8 bits of data from DR
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}else{
						// Parity is enabled
						// read only 7 bits from the DR
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}
			if(pUSARTHandle->RxLen == 0){
				// disable the RXNEIE
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;

				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	// Check for the CTS flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3){
		// Clear the CTS flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	// Check for the IDLE detection flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2){
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	// Check for the Overrun detection flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}

	// Check for the error flag
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);
	if(temp2){
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE)){
			// Framing Error
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}
		if(temp1 & (1 << USART_SR_NF)){
			// Noise Error
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NF);
		}
		if(temp1 & (1 << USART_SR_ORE)){
			// Overrun Error
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}


























