/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: May 16, 2020
 *      Author: Nabli Hatem
 */

//#include "stm32f303xx.h"
#include "stm32f303xx_gpio_driver.h"


/*******************************************************************
 *********************Peripheral Clock setup************************
 *******************************************************************
 * @fn			- GPIO PeriClockControl
 *
 * @brief		- this function enables or disables peripheral clock for given GOIO port
 *
 * @param[in]	- base address of the GPIO peripheral
 * @param[in]	- Enable or disable macros
 * @param[in]
 *
 * @return		- none
 *
 * @note		- none
 *
 ********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}
/************************************************************************
 ***************************** GPPIO Init  ******************************
 ************************************************************************
 *
 *@fn			- GPIO_Init
 *
 *@brief		- this function initialise peripheral parameter
 *
 *@param[in]	- GPIO pinConfig structure and peripheral base address
 *@param[in]	-
 *
 *@return		- none
 *
 *
 *************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp;
	//1 . configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
		pGPIOHandle->pGPIOx->MODER |= temp; //Setup

	}
	else
	{
		//this part will code later (interrupt mode)
	}

	temp=0;
	//2 . configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//3 . configure the pupd setting

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4 . configure the optype
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //Clear
	pGPIOHandle->pGPIOx->OTYPER |=temp;
	temp = 0;
	//5 . configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));

	}


}
/************************************************************************
 ****************************** GPIO DeInit *****************************
 ************************************************************************
 *
 *@fn			- GPIO_DeInit
 *
 *@brief		- this function Desinitialise peripheral parameter
 *
 *@param[in]	- peripheral base address of the given GPIO
 *@param[in]	-
 *
 *@return		- none
 *
 *
 *************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/************************************************************************
 *********************** Data read and write ****************************
 ************************************************************************
 *
 *@fn			- GPIO_ReadFromInputPin
 *
 *@brief		- this function read input pin of the given gpio
 *
 *@param[in]	- peripheral base address of the given GPIO
 *@param[in]	- pin number
 *
 *@return		- 0 or 1
 *
 *
 *************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


uint16_t GPIO_ReadFrom_InputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t  Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi){

}
void GPIO_IRGHandling(uint8_t PinNumber){

}


