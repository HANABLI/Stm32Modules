/*
 * 002main.c
 *
 *  Created on: May 18, 2020
 *      Author: Nabli Hatem
 */


/*
 * mainLedToggle.c
 *
 *  Created on: May 18, 2020
 *      Author: Nabli Hatem
 */
/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif*/

#include "stm32f303xx.h"
//#include "stm32f303xx_gpio_drivers.h"
void delay()
{
	for(uint32_t i=0;i<50000/2;i++);
}
int main(void)
{
	GPIO_Handle_t LedTest;
	LedTest.pGPIOx = GPIOA;
	LedTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	LedTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LedTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_FAST;
	LedTest.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	LedTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&LedTest);


	GPIO_Handle_t ButtonTest;
		ButtonTest.pGPIOx = GPIOC;
		ButtonTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		ButtonTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		ButtonTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_FAST;
		ButtonTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;


		GPIO_Init(&ButtonTest);

	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13))
		{
			delay();
			GPIO_ToggleOutputPin(LedTest.pGPIOx, LedTest.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

}

