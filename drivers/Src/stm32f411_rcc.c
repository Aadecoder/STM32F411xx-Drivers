#include "stm32f411.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};


/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - Gets the Peripheral Clock value of apb1 bus
 *
 * @param[in]         - None
 *
 * @return            - pclk1 value
 *
 * @Note              - None

 *********************************************************************/
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// for apb1

	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4){
		apb1p = 1;
	}else{
		apb1p = APB_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp ) / apb1p;

	return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - Gets the Peripheral Clock value of apb2 bus
 *
 * @param[in]         - None
 *
 * @return            - pclk2 value
 *
 * @Note              - None

 *********************************************************************/
uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2, temp, SystemClk;

	uint8_t clksrc, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	temp = (RCC->CFGR >> 13 ) & 0x7;
	if(temp < 0x04){
		apb2p = 1;
	}else{
		apb2p = APB_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp )/ apb2p;

	return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void){
	// Not Implemented
	return 0;
}






















