/*
 * it.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */

#include "stm32f3xx_hal.h"
extern UART_HandleTypeDef huart2;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}
