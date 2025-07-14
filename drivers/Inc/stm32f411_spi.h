#ifndef INC_STM32F411_SPI_H_
#define INC_STM32F411_SPI_H_

#include "stm32f411.h"


// SPI Configuration Structure
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
}SPI_Config_t;

// SPI Handle Structure
typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;			// To store the app. Tx buffer address
	uint8_t *pRxBuffer;			// To Store the app. Rx buffer address
	uint32_t TxLen; 			// To store Tx Len
	uint32_t RxLen; 			//  To Store Rx Len
	uint8_t TxState; 			// To store Tx state
	uint8_t RxState; 			// To store Rx State
}SPI_Handle_t;

// SPI API Prototypes

// Peripheral Clock Control
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// SPI Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//SPI Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


// IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

// Other Peripheral Control API's

// GET FLAG STATUS
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// SPI Peripheral Enable/Disable
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// SPI SSI Config
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// SPI SSOE Config
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// SPI Clear Overrun Flag
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

// SPI Close Transmission
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

// SPI Close Reception
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

// SPI Application Callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

// Macros for Peripheral Configuration
// 1. @SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

// 2. @SPI_BusConfig
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

// 3. @SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2         0
#define SPI_SCLK_SPEED_DIV4         1
#define SPI_SCLK_SPEED_DIV8         2
#define SPI_SCLK_SPEED_DIV16        3
#define SPI_SCLK_SPEED_DIV32        4
#define SPI_SCLK_SPEED_DIV64        5
#define SPI_SCLK_SPEED_DIV128       6
#define SPI_SCLK_SPEED_DIV256       7

// 4. @SPI_DFF
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

// 5. @SPI_CPHA
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

// 6. @SPI_CPOL
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

// 7. @SPI_SSM
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0




















#endif /* INC_STM32F411_SPI_H_ */
