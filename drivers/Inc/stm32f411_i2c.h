#ifndef INC_STM32F411_I2C_H_
#define INC_STM32F411_I2C_H_

#include "stm32f411.h"

// I2C Configuration Structure
typedef struct{
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

// I2C Handle Structure
typedef struct{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t *pTxBuffer;       // to store application tx buffer address
    uint8_t *pRxBuffer;       // to store application rx buffer address
    uint32_t TxLen;           // to store the tx len
    uint32_t RxLen;           // to store the rx len
    uint8_t TxRxState;        // to store the communication state
    uint8_t DevAddr;          // to store slave device address
    uint32_t RxSize;           // to store the reception size
    uint8_t Sr;               // to store repeated Start value
}I2C_Handle_t;

// User Configurable Macros
// @I2C_SCLSpeed
#define I2C_SCL_SPEED_SM            100000
#define I2C_SCL_SPEED_FM4K          400000
#define I2C_SCL_SPEED_FM2K          200000

// @I2C_ACKControl
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0

// @I2C_FMDutyCycle
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

// I2C Application States
#define I2C_READY           0
#define I2C_BUSY_IN_TX      1
#define I2C_BUSY_IN_RX      2


// I2C API Prototypes

// Peripheral Clock Control
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// I2C Init/ De-Init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// I2C Data Send and receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

// IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQInterruptPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

// Other Control API's
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

// Application Callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

// I2C Related Status Flag Definations
#define I2C_FLAG_TXE      ( 1 << I2C_SR1_TXE )
#define I2C_FLAG_RXNE     ( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_OVR      ( 1 << I2C_SR1_OVR )
#define I2C_FLAG_SB       ( 1 << I2C_SR1_SB )
#define I2C_FLAG_ADDR     ( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_BTF      ( 1 << I2C_SR1_BTF )
#define I2C_FLAG_STOPF    ( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_BERR     ( 1 << I2C_SR1_BERR )
#define I2C_FLAG_ARLO     ( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_AF       ( 1 << I2C_SR1_AF )
#define I2C_FLAG_TIMEOUT  ( 1 << I2C_SR1_TIMEOUT )

// Repeted Start Macros
#define I2C_DISABLE_SR  RESET
#define I2C_ENABLE_SR     SET



























#endif /* INC_STM32F411_I2C_H_ */
