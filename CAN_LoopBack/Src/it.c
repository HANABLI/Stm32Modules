/*
 * it.c
 *
 *  Created on: Jun 24, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f3xx_hal.h"

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
