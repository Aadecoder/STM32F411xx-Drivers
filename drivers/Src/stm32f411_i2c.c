#include "stm32f411.h"

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
//uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
//uint8_t APB1_PreScalar[4] = {2, 4, 8, 16};
//uint32_t RCC_GetPCLK1Value(void){
//    uint32_t pclk1, SystemClk;
//    uint8_t clksrc, temp, ahbp, apb1p;
//
//    clksrc = ((RCC->CFGR >> 2) & (0x3));
//
//    if(clksrc == 0){
//        // HSI Clock Source
//        SystemClk = 16000000; // System Clock value is 16MHz
//    }else if(clksrc == 1){
//        // HSE Clock Source
//        SystemClk = 8000000; // System Clock value is 8MHz
//    }
//
//    // For AHB
//
//}
