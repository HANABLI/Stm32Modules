/*
 * it.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */
#include "it.h"

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
