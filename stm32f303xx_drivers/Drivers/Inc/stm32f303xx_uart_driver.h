/*
 * stm32f303xx_uart_driver.h
 *
 *  Created on: Jun 5, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_UART_DRIVER_H_
#define INC_STM32F303XX_UART_DRIVER_H_
#include "stm32f303xx.h"

typedef struct
{
	uint8_t USART_ModeConf	;
	uint32_t USART_BaudConf;
	uint8_t	USART_NoOfStopBits;
	uint32_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct
{
	USART_Config_t USART_Config;
	USART_RegDef_t *pUSARTx;
}USART_Handle_t;
/*
 * @USART_Mode
 * Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TXRX			2
/*
 * @USART_Baud
 * Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200				1200
#define USART_STD_BAUD_2400				2400
#define USART_STD_BAUD_9600				9600
#define USART_STD_BAUD_19200			19200
#define USART_STD_BAUD_38400			38400
#define USART_STD_BAUD_57600			57600
#define USART_STD_BAUD_115200			115200
#define USART_STD_BAUD_230400			230400
#define USART_STD_BAUD_460800			460800
#define USART_STD_BAUD_921600			921600
#define USART_STD_BAUD_2M				2000000
#define USART_STD_BAUD_3M				3000000

/*
 * @USART_ParityControl
 * Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD				2
#define USART_PARITY_EN_EVEN			1
#define	USART_PARITY_DISABLE			0

/*
 * @USART_WordLength
 * Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1
#define USART_WORDLEN_7BITS			2

/*
 * @USART_NoOfStopBits
 * possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5			3

/*
 * @USART_HWFlowControl
 * Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/*
 * FlagStatus
 */

#define USART_TC_FLAG				(1 << USART_ISR_TC)
#define USART_RXNE_FLAG				(1 << USART_ISR_RXNE)
#define USART_TXE_FLAG				(1 << USART_ISR_TXE)
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_PeriControle(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and DeInit
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data send and receive
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_MasterSendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t Len);
uint8_t USART_MasterReadDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t Len);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t statusFlageNAme);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQEventHandling(USART_Handle_t *pUSARTHandle);


/*
 * Application callback
 */

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHAndle, uint8_t appEv);
#endif /* INC_STM32F303XX_UART_DRIVER_H_ */
