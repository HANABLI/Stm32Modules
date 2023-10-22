/*
 * stm32f303xx_uart_driver.c
 *
 *  Created on: Jun 5, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f303xx_uart_driver.h"
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}
}
void USART_PeriControle(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);

	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//variable to hold PCLK value
	uint32_t PCLKx;
	uint32_t usartdiv;
	//variable to hold MAntissa and fraction values
	uint32_t M_part, F_part;
	uint32_t tempreg = 0;

	//Get the value of APB bus clock in the variable PCLKx
	if(pUSARTx == USART1)
	{
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}
	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((100* PCLKx)/(8*BaudRate));
	}else
	{
		usartdiv = ((100* PCLKx)/(16*BaudRate));
	}

	M_part = usartdiv/100;

	tempreg|= M_part << 4;

	F_part = usartdiv - (M_part * 100);
	//Calculate the final fractional
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 over sampling by 8
		F_part = (((F_part * 8)+ 50)/100) & ((uint8_t)0x07);
	}
	else
	{
		F_part = (((F_part * 16)+ 50)/100) & ((uint8_t)0x0F);
	}

	tempreg|= F_part;

	pUSARTx->BRR |= tempreg;
}
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable the peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	//Set the peripheral Mode
	if(pUSARTHandle->USART_Config.USART_ModeConf == USART_MODE_ONLY_RX )
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_ModeConf == USART_MODE_ONLY_TX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_ModeConf == USART_MODE_TXRX)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
	}

	// Implement the code to configure the Word length configuration item
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_M1);
	}
	else
	{
		pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M0);
	}



	// Implement the code to configure the parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);

	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PS);
	}
	//Set NoOfStopBit
	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);
	pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_DMAT);
	//Implement the code to configure the hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 |= (1 << USART_CR3_RTSE);
	}
	else
	{
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << USART_CR3_CTSE);
		pUSARTHandle->pUSARTx->CR3 &= ~(1 << USART_CR3_RTSE);
	}
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudConf);
}
void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
	for(int i =0;i<Len;i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_TXE_FLAG));

		//Check the USART_WordLength item for 9BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//this is 8bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}
	//Implement the code to wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG));
}
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{

}
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
 if(pUSARTx->ISR & FlagName)
 {
	 return FLAG_SET;
 }
 return FLAG_RESET;
}
