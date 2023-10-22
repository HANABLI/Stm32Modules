/*
 * msp.c
 *
 *  Created on: Jun 24, 2020
 *      Author: Nabli Hatem
 */

#include "stm32f3xx_hal.h"
void HAL_MspInit(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	SCB->SHCSR |= 0x7 << 16;

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);



}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpio_uart;
  // here we are going to do the low level inits. of the USART2 peripheral

	//1. enable the clock for the USART2 peripheral
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//2. Do the pin muxing configurations

	gpio_uart.Pin = GPIO_PIN_2;
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULLUP;
	gpio_uart.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_uart.Alternate = GPIO_AF7_USART2; //UART2_TX
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	gpio_uart.Pin = GPIO_PIN_3; //UART2_RX
	HAL_GPIO_Init(GPIOA, &gpio_uart);
	//3. Enable the IRQ and set up the priority (NVIC setting)
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStract;

	__HAL_RCC_CAN1_CLK_ENABLE();

	/**CAN1 GPIO Configuration
	 * PA11 ----> CAN1_RX
	 * PA12 ----> CAN1_TX
	 */
	GPIO_InitStract.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStract.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStract.Pull = GPIO_NOPULL;
	GPIO_InitStract.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStract.Alternate = GPIO_AF9_CAN;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStract);

}
