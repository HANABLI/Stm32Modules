/*
 * stm32f303xx_spi_driver.c
 *
 *  Created on: May 21, 2020
 *      Author: Nabli Hatem
 */
#include <stdio.h>
#include "stm32f303xx.h"
static void spi_txe_interrupt_handle(SPI_Handle_t *SPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *SPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *SPIHandle);
/*
 * Peripheral Clock setup
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void SPI_PeriControle(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register


	//uint32_t tempreg = 0 ;
	//0. SPE Enable


	pSPIHandle->pSPIx->CR1  |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR ;
	//2. configure bus
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		pSPIHandle->pSPIx->CR1  &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		pSPIHandle->pSPIx->CR1  |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX)
	{
		pSPIHandle->pSPIx->CR1  &= ~(1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1  |= (1 << SPI_CR1_RXONLY);
	}
	//3. speed configuration

	pSPIHandle->pSPIx->CR1  |= pSPIHandle->SPI_Config.SPI_ClkSpeed << SPI_CR1_BR;


	//5. CPHA config

	pSPIHandle->pSPIx->CR1  |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//6. CPOL config

	pSPIHandle->pSPIx->CR1  |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
	//7. SSM config

	pSPIHandle->pSPIx->CR1  |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	//1. configure Device Mode


	//tempreg = 0;
	//4. DFF config

	pSPIHandle->pSPIx->CR2  |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR2_DS;
	//pSPIHandle->pSPIx->CR2 = tempreg;
}
void SPI_SSIConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t * pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<< SPI_CR2_SSOE);
	}
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_REG_RESET();
		}
}
/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);
		if(pSPIx->CR2 & (SPI_DS_8BITS << SPI_CR2_DS))
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;

		}
		else if(pSPIx->CR2 & (SPI_DS_16BITS << SPI_CR2_DS))
		{
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			Len--;
			Len--;

			(uint16_t*)pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if(pSPIx->CR2 & (SPI_DS_8BITS << SPI_CR2_DS))
		{
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;

		}
		else if(pSPIx->CR2 & (SPI_DS_16BITS << SPI_CR2_DS))
		{
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
	}
}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	//4. Data Transmission will be handled by the ISR code

	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXEIE);
	}
	//4. Data Transmission will be handled by the ISR code

	return state;
}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (FLAG_SET << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (FLAG_SET << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (FLAG_SET << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (FLAG_SET << SPI_CR2_RXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (FLAG_SET << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (FLAG_SET << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR2 & (SPI_DS_8BITS << SPI_CR2_DS))
			{
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer ;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;

			}
			else if(pSPIHandle->pSPIx->CR2 & (SPI_DS_16BITS << SPI_CR2_DS))
			{
				pSPIHandle->pSPIx->DR = *(uint16_t*)pSPIHandle->pRxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
	if(! pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR2 & (SPI_DS_8BITS << SPI_CR2_DS))
			{
				*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;

			}
			else if(pSPIHandle->pSPIx->CR2 & (SPI_DS_16BITS << SPI_CR2_DS))
			{
				*(uint16_t*)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
	if(! pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//
}
