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













































