/*
 * stm32f303xx_i2c_drivers.h
 *
 *  Created on: Jun 1, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_I2C_DRIVERS_H_
#define INC_STM32F303XX_I2C_DRIVERS_H_

#include "stm32f303xx.h"
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;


}I2C_Config_t;

typedef struct
{
	I2C_Config_t I2C_Config;
	I2C_RegDef_t *pI2Cx;
	uint8_t	*pTxBuffer;		/* !< To store the app. Tx buffer address >*/
	uint8_t *pRxBuffer;		/* !< To store the app. Rx buffer address >*/
	uint32_t TxLen;			/* !< To store Tx Len >*/
	uint32_t RxLen;			/* !< To store Rx Len >*/
	uint8_t  TXRXState;		/* !< To store Communication state>*/
	uint8_t DevAddr;		/* !< To store slave/device address>*/
	uint32_t RxSize;		/* !< To store Rx size>*/
	uint8_t Sr;				/*!< To store repeated start value>*/
}I2C_Handle_t;

/*
 * I2C applications states
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2
/*Analog delay filter ON
 * Coefficient of digital filter 0
 * Rise Time		100ns
 * Fall Time 		50ns
 */
#define I2C_SCL_HSI					 8000000
#define I2C_SCL_HSE					16000000
/*
 * @I2C_SCLSpeed 8MHz
 */
#define I2C_SCL8_SPEED_SM10			(uint32_t)0x10108CFF
#define I2C_SCL8_SPEED_SM100		(uint32_t)0x00301D2A
#define I2C_SCL8_SPEED_FM4K			(uint32_t)0x0010020A
#define I2C_SCL8_SPEED_FM2K			(uint32_t)0x0010021E

#define I2C_SCL_SPEED_SM10			0
#define I2C_SCL_SPEED_SM100			1
#define I2C_SCL_SPEED_FM4K			2
#define I2C_SCL_SPEED_FM2K			3
/*
 * @I2C_SCLSpeed 16MHz
 */
#define I2C_SCL16_SPEED_SM10			(uint32_t)0x30108DFF
#define I2C_SCL16_SPEED_SM100			(uint32_t)0x00503D59
#define I2C_SCL16_SPEED_FM4K			(uint32_t)0x00300618
#define I2C_SCL16_SPEED_FM2K			(uint32_t)0x00300640
/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE				RESET
#define I2C_ACK_DISABLE				ENABLE
/*
 * @I2C_FMDutyCycle
 */



/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
/*
 * Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
/*
 * Data send and receive
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pSPIx, uint8_t FlagName);

uint8_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);
uint8_t I2C_MasterReadData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReadDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle);
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t event);
#endif /* INC_STM32F303XX_I2C_DRIVERS_H_ */
