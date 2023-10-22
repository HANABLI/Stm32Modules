/*
 * msp.c
 *
 *  Created on: Jun 19, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f3xx_hal.h"
void HAL_MspInit(void)
{
	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	//2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16; // usage fault, memory fault and bus fault system exception
	//3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
	HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
	HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	/*
	 *
	 * pa0 -> ch1
	 * pa1 -> ch2
	 * pb10 -> ch3
	 * pb2 -> ch4
	 */
	GPIO_InitTypeDef pgpio_tim2oc;
	/* 1. enable the peripheral clock for the timer2 peripheral*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();


	/* 2. Do the pin muxing configurations */

	pgpio_tim2oc.Pin = GPIO_PIN_0|GPIO_PIN_1;
	pgpio_tim2oc.Mode = GPIO_MODE_AF_PP;
	pgpio_tim2oc.Pull = GPIO_NOPULL;
	pgpio_tim2oc.Alternate = GPIO_AF1_TIM2;
	pgpio_tim2oc.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &pgpio_tim2oc);

	pgpio_tim2oc.Pin = GPIO_PIN_11|GPIO_PIN_10;
	pgpio_tim2oc.Mode = GPIO_MODE_AF_PP;
	pgpio_tim2oc.Pull = GPIO_NOPULL;
	pgpio_tim2oc.Alternate = GPIO_AF1_TIM2;
	pgpio_tim2oc.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &pgpio_tim2oc);

	/* 3. nvic setting*/
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM2_IRQn, 15 ,0);

}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef pgpio_uart;

	//her we are going to do the low level init of the UART2 peripheral
	/* 1. enable the clock for the USART2 peripheral*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	/* 2. Do the pin muxing configurations*/

	pgpio_uart.Pin = GPIO_PIN_2;
	pgpio_uart.Mode = GPIO_MODE_AF_PP;
	pgpio_uart.Pull = GPIO_PULLUP;
	pgpio_uart.Speed = GPIO_SPEED_FREQ_HIGH;
	pgpio_uart.Alternate = GPIO_AF7_USART2; //UART2_TX
	HAL_GPIO_Init(GPIOA, &pgpio_uart);

	pgpio_uart.Pin = GPIO_PIN_3; //UART2_RX
	HAL_GPIO_Init(GPIOA, &pgpio_uart);
	//3. Enable the IRQ and set up the priority (NVIC setting)
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);

}


