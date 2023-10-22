/*
 * it.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */

#include "stm32f3xx_hal.h"
extern TIM_HandleTypeDef htimer6;
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);
}
