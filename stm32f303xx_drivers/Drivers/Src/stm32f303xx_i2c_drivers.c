/*
 * stm32f303xx_i2c_drivers.c
 *
 *  Created on: Jun 3, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f303xx_i2c_drivers.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
/*
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );
*/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= ( 1 << I2C_CR2_START);
}
/*
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{

}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{

}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{

}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{

}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

}*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}

	}
}
void I2C_PeriControle(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	I2C_PeriControle(pI2CHandle->pI2Cx, DISABLE);
	// Make sure I2C1 is disabled
	pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);

	// Reset I2C1 Configuration to default values
	//pI2CHandle->pI2Cx->CR1 	  	= 0x00000000;
	//pI2CHandle->pI2Cx->CR2 	  	= 0x00000000;
	//pI2CHandle->pI2Cx->TIMINGR 	= 0x00000000;
	//Ack control bit
	pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->I2C_Config.I2C_AckControl << 15);
	//Configure Device address
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 |= (1 << 15);
	//Configure TIMINGR
	if(RCC_GetPCLKValue()==I2C_SCL_HSI)
	{
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM10)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL8_SPEED_SM10;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM100)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL8_SPEED_SM100;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL8_SPEED_FM2K;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL8_SPEED_FM4K;
		}
	}
	else
	{
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM10)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL16_SPEED_SM10;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM100)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL16_SPEED_SM100;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL16_SPEED_FM2K;
		}
		else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K)
		{
			pI2CHandle->pI2Cx->TIMINGR |= I2C_SCL16_SPEED_FM4K;
		}
	}
	//Enable I2C Peripheral


}
uint8_t I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	uint32_t 	timeout;	// Flag waiting timeout
	uint8_t		n;		// Loop counter
	//1. Set device address
	//pI2CHandle->pI2Cx->CR2 |= (0b0000000000 << I2C_CR2_SADDS);
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << I2C_CR2_SADD);
	//2. Set I2C write mode
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);
	//3. Transfer NBYTES, with AUTOEND

	//pI2CHandle->pI2Cx->CR2 |= (0b00000000 << I2C_CR2_NBYTES);
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
	//4.clear stop flag
	I2C1->ICR |= I2C_ICR_STOPCF;
	//5. Generate the START condition
	I2C_PeriControle(pI2CHandle->pI2Cx, ENABLE);
	//Enable I2C Peripheral
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ISR_ADDR)   );

	n = Len;
	while(n>0)
	{
		// Wait for TXIS with timout
		timeout = 100000;
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ISR_TXIS))
		{
			timeout --;
			if(timeout == 0) return 1;
		}
		//Send data
		pI2CHandle->pI2Cx->TXDR = *pTxbuffer;
		pTxbuffer++;
		n--;
	}
	// Wait for STOPF with timeout
	timeout = 100000;
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ISR_STOPF) )
	{
		timeout--;
		if (timeout == 0) return 3;
	}
	return 0;
}
uint8_t I2C_MasterReadData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	uint32_t 	timeout;	// Flag waiting timeout
		uint8_t		n;		// Loop counter
		//1. Set device address
		pI2CHandle->pI2Cx->CR2 |= (0b0000000000 << I2C_CR2_SADD);
		pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << I2C_CR2_SADD);
		//2. Set I2C read mode
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);
		//3. Transfer NBYTES, with AUTOEND

		pI2CHandle->pI2Cx->CR2 |= (0b00000000 << I2C_CR2_NBYTES);
		pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);
		pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
		//4.clear stop flag
		I2C1->ICR |= I2C_ICR_STOPCF;
		//5. Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		n = Len;
		while(n>0)
		{
			// Wait for RXNE with timout
			timeout = 100000;
			while(!(pI2CHandle->pI2Cx->ISR & (I2C_ISR_RXNE)))
			{
				timeout --;
				if(timeout == 0) return 1;
			}
			//Send data
			*pTxbuffer = pI2CHandle->pI2Cx->RXDR ;
			pTxbuffer++;
			n--;
		}
		// Wait for STOPF with timeout
			timeout = 100000;
			while (!((pI2CHandle->pI2Cx->ISR) & I2C_ISR_STOPF))
			{
				timeout--;
				if (timeout == 0) return 3;
			}
			return 0;
}
void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->TXDR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->RXDR;
}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TXRXState;
	if((busystate != I2C_BUSY_IN_TX)&&(busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TXRXState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		//Implement code to generate statrt condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable TXIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TXIE);
		//Implement the code to enable ERRIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ERRIE);
		//Implement the code to enable RXIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_RXIE);
		//Implement the code to enable STOPIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOPIE);
		//Implement the code to enable TCIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_TCIE);
		//Implement the code to enable NACKIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_NACKIE);
		//Implement the code to enable ADDRIE Control bit
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ADDRIE);

	}
	return busystate;
}

uint8_t I2C_MasterReadDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	return 0;
}
void I2C_IRQEventHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_TXIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TXIS );
	//1. Handle for interrupt generated by TXIS event
	if(temp1 && temp2)
	{
		if(pI2CHandle->TXRXState == I2C_BUSY_IN_TX)
		{
			pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);

			pI2CHandle->TxLen--;

			pI2CHandle->pTxBuffer++;
		}
	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_RXIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_RXNE );
	//1. Handle for interrupt generated by RXNE event
	if(temp1 && temp2)
	{
		if(pI2CHandle->TXRXState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
				pI2CHandle->RxLen--;
			}

			if(pI2CHandle->RxSize >1)
			{
				if(pI2CHandle->RxLen ==2)
				{
					//clear the ack bit
					I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
				}

					//read
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
			}
			if(pI2CHandle->RxLen == 0)
			{
				//close i2c data
			}
		}
	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_STOPIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_STOPF );
	//1. Handle for interrupt generated by STOPF event
	if(temp1 && temp2)
	{
		//Clear the STOPF
		pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ISR_STOPF);
	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_TCIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TC );
	//1. Handle for interrupt generated by TC event
	if(temp1 && temp2)
	{

	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_TCIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TCR );
	//1. Handle for interrupt generated by TCR event
	if(temp1 && temp2)
	{

	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_ADDRIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_ADDR );
	//1. Handle for interrupt generated by TCR event
	if(temp1 && temp2)
	{
		//I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}
	temp1 = pI2CHandle->pI2Cx->CR1 & (1<< I2C_CR1_NACKIE);
	temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_NACKF );
	//1. Handle for interrupt generated by TCR event
	if(temp1 && temp2)
	{

	}
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR2 |= ( 1 << I2C_CR2_NACK);
	}else
	{
		//disable the ack
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_NACK);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName)
{
	if(pI2Cx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
