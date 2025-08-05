#include "stm32f411.h"

// Helper Functions Prototypes
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );

/********************************************************
* @fn                - I2C_PeriClockControl

* @brief             - This fucntion enables or disables peripheral clock for the given GPIO port

* @param[in]         - base address of the I2C peripheral
* @param[in]         - ENABLE or DISABLE macos

* @return              -  none

* @Note                - none
********************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
		if(EnorDi == ENABLE){
				if(pI2Cx == I2C1){ I2C1_PCLK_EN();}
				else if(pI2Cx == I2C2){ I2C2_PCLK_EN();}
				else if(pI2Cx == I2C3){ I2C3_PCLK_EN();}
		}
		else{
				if(pI2Cx == I2C1){ I2C1_PCLK_DI();}
				else if(pI2Cx == I2C2){ I2C2_PCLK_DI();}
				else if(pI2Cx == I2C3){ I2C3_PCLK_DI();}
		}
}


/********************************************************
* @fn                - RCC_GetPCLK1Value(void)

* @brief             - This fucntion calculates the PCLK1 value for the APB bus interface

* @param[in]         - none

* @return            -  PCLK1 value for the APB bus interface

* @Note              - none
********************************************************/
uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScalar[4] = {2, 4, 8, 16};
uint32_t RCC_GetPCLK1Value(void){
   uint32_t pclk1, SystemClk;
   uint8_t clksrc, temp, ahbp, apb1p;

   clksrc = ((RCC->CFGR >> 2) & (0x3));

   if(clksrc == 0){
       // HSI Clock Source
       SystemClk = 16000000; // System Clock value is 16MHz
   }else if(clksrc == 1){
       // HSE Clock Source
       SystemClk = 8000000; // System Clock value is 8MHz
   }

   // For AHB prescalar
   temp = ((RCC->CFGR >> 4) & 0xF);
   if(temp < 8){
	ahbp = 1;
   }else{
	ahbp = AHB_PreScalar[temp - 8];
   }

   // for APB Prescalar
   temp = ((RCC->CFGR >> 10) & 0X7);
   if (temp < 4){
	apb1p = 1;
   }else{
	apb1p = APB1_PreScalar[temp - 4];
   }
   
   pclk1 = (SystemClk / ahbp) / apb1p;

   return pclk1;

}



/********************************************************
* @fn                - I2C_Init

* @brief             - This fucntion initializes I2C pins

* @param[in]         - a pointer to the handle structure

* @return            -  none

* @Note              - none
********************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	// Enable the clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// ACK Control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// Configure the freq bit in CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = tempreg & 0x3F;

	// Program the device address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// Device in standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
	}else{
		// Device in Fast mode
		tempreg |= 1 << 15;
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			// for duty = 0
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			// for duty = 1
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// t rise configuration
	// storing the following value in Trise field ((Fpclk1 * Trise(max)) + 1)
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
			// Mode is standard mode
			// tempreg is trise value
			tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else {
			// Mode is fast mode
			tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F); 
}

/********************************************************
* @fn                - I2C_DeInit

* @brief             - This fucntion deinitializes I2C pins

* @param[in]         - base address of the I2C Port

* @return            -  none

* @Note              - none
********************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
			if(pI2Cx == I2C1){ I2C1_REG_RESET();}
			else if(pI2Cx == I2C2){ I2C2_REG_RESET();}
			else if(pI2Cx == I2C3){ I2C3_REG_RESET();}
}

/********************************************************
* @fn                - I2C_PeripheralControl

* @brief             - This function enables or disables the I2C Peripheral 

* @param[in]         - base address of the I2C Port
* @param[in]         - ENABLE OR DISABLE

* @return            - None

* @Note              - None
********************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
		if (EnorDi == ENABLE){
				pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		}else {
				pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
		}
}

/********************************************************
* @fn                - I2C_GetFlagStatus

* @brief             - This function returns the status of flags

* @param[in]         - base address of the Port
* @param[in]         - Flag Name

* @return            - uint8_t flag set or reset

* @Note              - Helper function
********************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
		if (pI2Cx->SR1 & FlagName){
				return FLAG_SET;
		}
		return FLAG_RESET;
}

/********************************************************
* @fn                - I2C_GenerateStartCondition

* @brief             - This function generates the start condition for I2C communication

* @param[in]         - base address of the Port

* @return            - None

* @Note              - Helper function
********************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START); 
}

/********************************************************
* @fn                - I2C_ExecuteAddressPhaseWrite

* @brief             - This function execute the Sending of address phase for Master Send Data

* @param[in]         - base address of the Port
* @param[in]         - Slave Address

* @return            - None

* @Note              - Helper function
********************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	// Clear the 0th bit
	SlaveAddr &= ~(1); // SlaveAddr is Slave Address + Read/Write bit
	// Put the data into the data register
	pI2Cx->DR = SlaveAddr;
}

/********************************************************
* @fn                - I2C_ExecuteAddressPhaseRead

* @brief             - This function execute the Sending of address phase for Master Receive Data

* @param[in]         - base address of the Port
* @param[in]         - Slave Address

* @return            - None

* @Note              - Helper function
********************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	// Clear the 0th bit
	SlaveAddr |= (1); // SlaveAddr is Slave Address + Read/Write bit
	// Put the data into the data register
	pI2Cx->DR = SlaveAddr;
}

/********************************************************
* @fn                - I2C_ManageAcking

* @brief             - This function closes acking

* @param[in]         - I2C Handle Structure
* @param[in]         - Enable / Disable

* @return            - None

* @Note              - Helper function
********************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/********************************************************
* @fn                - I2C_ClearAddrFlag

* @brief             - This function will clear the ADDR flag as the Address Phase is completed

* @param[in]         - I2C Handle Structure

* @return            - None

* @Note              - Helper function
********************************************************/
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;

	// Device is in master mode
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
		if(pI2CHandle->RxSize == 1){
			// First Disable the Acking
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

			// Clear the ADDR flag (read SR1 and SR2 registers)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}else{
		// Device is in Slave mode
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}

}

/********************************************************
* @fn                - I2C_Generate STOP condition

* @brief             - This function will generate STOP condition

* @param[in]         - I2C Base port address

* @return            - None

* @Note              - Helper function
********************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

/********************************************************
* @fn                - I2C_SendData

* @brief             - This function sends data

* @param[in]         - base address of the Port
* @param[in]         - Transmission Buffer
* @param[in]         - Length
* @param[in]         - Slave Address
* @param[in]         -

* @return            - None

* @Note              - None
********************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
	// 1. Generate Start Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that the start condition is completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of slave with the read/write bit set to 0
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to the software sequence
	I2C_ClearAddrFlag(pI2CHandle);

	// 6. Send the data until Len becomes 0
	while(Len>0){
		// Wait until the TXE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When the Len becomes zero wait for TXE and BTF flag to be equal to 1 before generating the STOP condition
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
	
	// 8. Generate the STOP condition
	if(Sr == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/********************************************************
* @fn                - I2C_MasterReceiveData

* @brief             - This function recives data for master device

* @param[in]         - I2C Handle Structure
* @param[in]         - Receive Buffer
* @param[in]         - Length
* @param[in]         - Slave Address

* @return            - None

* @Note              - None
********************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that the start condition is completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of slave with the read/write bit set to 1
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	  // 4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Produce the READ ONLY one byte from Slave
	if(Len == 1){
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear the ADDR Flag
		I2C_ClearAddrFlag(pI2CHandle);

		// wait until the RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Generate the STOP condition
		if(Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Read the data into the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	// Produce the read data from slave when Len > 1
	if(Len > 1){
		// Clear the ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		// read the data until the Len becomes zero
		for(uint32_t i = Len; i>0; i--){
			// wait until RXNE becomes 1
			if(i == 2){
				// if the last 2 bytes are remaining then clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate STOP condition
				if(Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			// read the data from the data register into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	// re enabling the ACKING
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/********************************************************
* @fn                - I2C_IRQInterruptConfig

* @brief             - Enabling or Disabling IRQ's in NVIC registers

* @param[in]         - IRQ Number
* @param[in]         - Enable or disable

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
		// Enabling or Disabling Interrupt on a given IRQ Line
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
						*NVIC_ICER0 |= ( 1 << IRQNumber);

				}else if(IRQNumber > 31 && IRQNumber < 64){
						// Program ICER1 Register
						*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));

				}else if(IRQNumber >= 64 && IRQNumber < 96){
						// Program ICER2 Register
						*NVIC_ICER2 |= ( 1 << (IRQNumber % 64));
				}
		}
}

/********************************************************
* @fn                - I2C_IRQPriorityConfig

* @brief             - Configure Priority Value for a given IRQ Number using Priotity Registers

* @param[in]         - IRQ Number
* @param[in]         - IRQ Priority

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
		// Finding out the IPR Register and Section
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;

		//configuring priority register
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/********************************************************
* @fn                - I2C_MasterSendDataIT

* @brief             - This function sends data with Interrupt capability

* @param[in]         - I2C Handle Variable address
* @param[in]         - Transmission Buffer
* @param[in]         - Length
* @param[in]         - Slave Address
* @param[in]         - Repeated start

* @return            - The I2C State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystate = pI2CHandle->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement the code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Enable the ITEVFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Enable the ITERREN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/********************************************************
* @fn                - I2C_ReceiveDataIT

* @brief             - This function receives data with Interrupt capability

* @param[in]         - I2C Handle Variable address
* @param[in]         - Rx Buffer
* @param[in]         - Length
* @param[in]         - Slave Address
* @param[in]         - Repeated start

* @return            - The I2C State

* @Note              - Non Blocking type transmission
********************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	// Disable the ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		// 1. Load the data into the DR
		pI2CHandle->pI2Cx->DR = pI2CHandle->pTxBuffer;

		// 2. Increment the buffer address
		pI2CHandle->pTxBuffer++;

		// 3. Decrement the Tx length
		pI2CHandle->TxLen--;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
		pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			// Clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		// read the DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}
	if(pI2CHandle->RxLen == 0){
		// 1. Generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR){
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		}
		// 2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/********************************************************
* @fn                - I2C_EV_IRQHandling

* @brief             -  Handles the interrupts triggered by events

* @param[in]         - IRQ Handle variable

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	// Interrupt Handling for both master and slave mode of the device
	uint8_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	// 1. Handle the interrupt generated by SB
	if(temp1 && temp3){
		// SB flag is set and the interrupt is generated by SB
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	// 2. Handle the Interrupt generated by ADDR event
	if(temp1 && temp3){
		// ADDR flag is set
		I2C_ClearAddrFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	// 3. Handle the interrupt generated by BTF event
	if(temp1 && temp3){
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			// Make sure the TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				// BTF and TXE are both set
				if(pI2CHandle->TxLen == 0){
					// 1. Generate the stop condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2. reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify the Application about the transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){;}

	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// 4. Handle the interrupt generated by STOPF condition
	// Stop detection flag is only applicable in Slave mode. for master this flag will never be set
	if(temp1 && temp3){
		// STOPF flag is set
		// Clear the STOPF flag
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		// Notify the application that the stop is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	// 5. Handle the interrupt generated by TXE event
	if(temp1 && temp2 && temp3){
		// Check for the device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// Master Mode (Controller Mode)
			// Do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else {
			// Slave Mode (Target Mode)
			// Make sure that the slave is really in Transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	// 6. Handle the interrupt generated by the RXNE event
	if(temp1 && temp2 && temp3){
		// Check for the device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// Master mode (Controller mode)
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else{
			// Slave mode (target mode)
			// Make sure that the slave is really in receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}

}

/********************************************************
* @fn                - I2C_ER_IRQHandling

* @brief             - Handles the interrupts triggered by errors

* @param[in]         - IRQ Handle variable

@return              - None

@Note                - Anything done here is processor specific
*********************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1 , temp2;

	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

	// 1. Check for BUS error
	temp1 = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR1_BERR);
	if(temp1 && temp2){
		// CLear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// 2. Check for the arbitration loss error
	temp1 = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2){
		// CLear the ARLO error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// 3. Check for the ACK failure error
	temp1 = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR1_AF);
	if(temp1 && temp2){
		// CLear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// 4. Check for the Overrun / Underrun error
	temp1 = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR1_OVR);
	if(temp1 && temp2){
		// CLear the overrun / under run error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// 5. Check for the timeout error
	temp1 = pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2){
		// CLear the timeout error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}


