#include "stm32f411.h"

// Helper Functions Prototypes
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
//static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
//static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/********************************************************
* @fn                - I2C_PeriClockControl

* @brief             - This fucntion enables or disables peripheral clock for the given GPIO port

* @param[in]         - base address of the I2C peripheral
* @param[in]         - ENABLE or DISABLE macos

* @return              -  none

* @Note                - none
********************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
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

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
		if (pI2Cx->SR1 & FlagName){
				return FLAG_SET;
		}
		return FLAG_RESET;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START); 
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	// the lsb is the read / write bit and must be set to 0 for write 
	// Clear the 0th bit or lsb

	SlaveAddr &= ~(1); // SlaveAddr = SlaveAddr + READ/WRITE bit
	
	// put the address in the data register
	pI2Cx->DR = SlaveAddr;
}

// static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle){
// 	uint32_t dummy_read;

// 	// Device is in master mode
// 	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
// 		if(pI2CHandle->RxSize == 1){
// 			// First disable the acking 

// 		}
		
// 	}

// }
