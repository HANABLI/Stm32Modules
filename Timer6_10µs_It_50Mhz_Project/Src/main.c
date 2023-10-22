/*
 * main.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f3xx_hal.h"
#include "main.h"
#define SYS_CLOCK_FREQ_24_MHZ	24
#define SYS_CLOCK_FREQ_48_MHZ	48
#define SYS_CLOCK_FREQ_72_MHZ	72
#define TRUE	1
#define FALSE	0
#include <string.h>


void Error_handler(void);
void Sysclk_Config(uint8_t freq);
void TIMER6_Init(void);
void GPIO_Init(void);
TIM_HandleTypeDef htimer6;

int main(void)
{
	HAL_Init();

	Sysclk_Config(SYS_CLOCK_FREQ_48_MHZ);
	GPIO_Init();
	TIMER6_Init();

	//Lets start timer
	HAL_TIM_Base_Start_IT(&htimer6);


	while(1);

	return 0;
}

void Sysclk_Config(uint8_t freq)
{

	uint8_t latency = 0;
		RCC_OscInitTypeDef osc_init;
		RCC_ClkInitTypeDef clk_init;

		osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		osc_init.HSIState = RCC_HSI_ON;
		osc_init.HSICalibrationValue = 16;
		osc_init.PLL.PLLState = RCC_PLL_ON;
		osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

		switch(freq)
		{
			case SYS_CLOCK_FREQ_24_MHZ:
			{
				osc_init.PLL.PREDIV = RCC_CFGR2_PREDIV_DIV3;
				osc_init.PLL.PLLMUL = RCC_CFGR_PLLMUL9;
				latency = FLASH_LATENCY_0;
				break;
			}
			case SYS_CLOCK_FREQ_48_MHZ:
			{
				osc_init.PLL.PREDIV = RCC_CFGR2_PREDIV_DIV2;
				osc_init.PLL.PLLMUL = RCC_CFGR_PLLMUL12;
				latency = FLASH_LATENCY_1;
				break;
			}
			case SYS_CLOCK_FREQ_72_MHZ:
			{
				osc_init.PLL.PREDIV = RCC_CFGR2_PREDIV_DIV1;
				osc_init.PLL.PLLMUL = RCC_CFGR_PLLMUL9;
				latency = FLASH_LATENCY_2;
				break;
			}
			default:
				return ;
		}
		if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
		{
			Error_handler();
		}
		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
							RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		if(HAL_RCC_ClockConfig(&clk_init, latency)!= HAL_OK)
		{
			Error_handler();
		}
		//Systick configuration
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void TIMER6_Init(void)
{
	//__HAL_RCC_TIM6_CLK_ENABLE();
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 9;
	htimer6.Init.Period = 48-1;
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
