/*
 * main.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f3xx_hal.h"
#include "main.h"
//#include <string.h>


void Error_handler(void);
void Sysclk_Config(void);
void TIMER6_Init(void);
void GPIO_Init(void);
TIM_HandleTypeDef htimer6;

int main(void)
{
	HAL_Init();

	Sysclk_Config();
	GPIO_Init();
	TIMER6_Init();

	//Lets start timer
	HAL_TIM_Base_Start_IT(&htimer6);


	while(1);

	return 0;
}

void Sysclk_Config(void)
{


}

void TIMER6_Init(void)
{
	//__HAL_RCC_TIM6_CLK_ENABLE();
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 15;
	htimer6.Init.Period = 50000-1;
	if(HAL_TIM_Base_Init(&htimer6)!= HAL_OK)
	{
		Error_handler();
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
void GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	ledgpio.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOA, &ledgpio);
}
void Error_handler(void)
{
	while(1);
}
