/*
 * stm32f303xx_spi_driver.h
 *
 *  Created on: May 21, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_SPI_DRIVER_H_
#define INC_STM32F303XX_SPI_DRIVER_H_
#include "stm32f303xx.h"

typedef struct
{
	uint32_t SPI_DeviceMode;		/*!< Slave or Master mode> */
	uint32_t SPI_BusConfig;			/* !<Configuration bus Full duplex or half duplex or simplex>*/
	uint32_t SPI_ClkSpeed; 			/*!<Clock speed>*/
	uint32_t SPI_DFF;				/*!< data frame format>*/
	uint32_t SPI_CPHA;
	uint32_t SPI_CPOL;
	uint32_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_Config_t SPI_Config;
	SPI_RegDef_t *pSPIx;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;


/*
 * SPI application states
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2
/*
 * Possible SPI Application events
 */

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX				3

/*
 * @SPI_ClkSpeed
 */
#define SPI_CLK_SPEED_DIV2					0
#define SPI_CLK_SPEED_DIV4					1
#define SPI_CLK_SPEED_DIV8					2
#define SPI_CLK_SPEED_DIV16					3
#define SPI_CLK_SPEED_DIV32					4
#define SPI_CLK_SPEED_DIV64					5
#define SPI_CLK_SPEED_DIV128				6
#define SPI_CLK_SPEED_DIV256				7



/*
 * @SPI_DFF
 */

#define SPI_DS_4BITS						3
#define SPI_DS_5BITS						4
#define SPI_DS_6BITS						5
#define SPI_DS_7BITS						6
#define SPI_DS_8BITS						7
#define SPI_DS_9BITS						8
#define SPI_DS_10BITS						9
#define SPI_DS_11BITS						10
#define SPI_DS_12BITS						11
#define SPI_DS_13BITS						12
#define SPI_DS_14BITS						13
#define SPI_DS_15BITS						14
#define SPI_DS_16BITS						15


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI							0
#define	SPI_SSM_EN							1
/*
 * @SPI_SPE
 */
#define SPI_SPE_EN							1
#define SPI_SPE_DI							0
/*
 * FlagStatus
 */

#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_PeriControle(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data send and receive
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);
void SPI_SSIConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * Application callback
 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F303XX_SPI_DRIVER_H_ */
