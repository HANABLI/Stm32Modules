/*
 * main.c
 *
 *  Created on: Jun 19, 2020
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
#include <stdio.h>


void UART2_Init(void);
void TIMER2_Init(void);
void Error_handler(void);
void Sysclk_Config(uint8_t freq);

TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;

uint32_t pulse1_value = 24000; 	/*to produce 500Hz*/
uint32_t pulse2_value = 12000; 	/*to produce 1000Hz*/
uint32_t pulse3_value = 6000; 	/*to produce 2000Hz*/
uint32_t pulse4_value = 3000; 	/*to produce 4000Hz*/
uint32_t ccr_content;
int main(void)
{
	HAL_Init();

	Sysclk_Config(SYS_CLOCK_FREQ_48_MHZ);
	//UART2_Init();
	TIMER2_Init();
	if(HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_1) != HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_2) != HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_3) != HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_4) != HAL_OK)
	{
		Error_handler();
	}
	while(1);
	return 0;
}
void Sysclk_Config(uint8_t freq)
{

	uint8_t latency = 0;
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_ON;
	//osc_init.HSECalibrationValue = 16;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

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
void TIMER2_Init(void)
{
	TIM_OC_InitTypeDef timer2OC_Config;
	htimer2.Instance = TIM2;
	htimer2.Init.Period = 0xFFFFFFFF;
	htimer2.Init.Prescaler = 1;
	if(HAL_TIM_OC_Init(&htimer2)!= HAL_OK)
	{
		Error_handler();
	}
	timer2OC_Config.OCMode = TIM_OCMODE_TOGGLE;
	timer2OC_Config.OCPolarity = TIM_OCNPOLARITY_HIGH ;


	timer2OC_Config.Pulse = pulse1_value;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_handler();
	}
	timer2OC_Config.Pulse = pulse2_value;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_handler();
	}
	timer2OC_Config.Pulse = pulse3_value;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_handler();
	}
	timer2OC_Config.Pulse = pulse4_value;
	if(HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_handler();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM3_CH1 toggling with frequency = 500Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,ccr_content+pulse1_value);
	}
	/* TIM2_CH2 toggling wih frequency = 1000 Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,ccr_content+pulse2_value);
	}
	/* TIM2_CH3 toggling wih frequency = 1000 Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,ccr_content+pulse3_value);
	}
	/* TIM2_CH4 toggling wih frequency = 1000 Hz */
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,ccr_content+pulse4_value);
	}
}
void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if(HAL_UART_Init(&huart2)!=HAL_OK)
	{
		//There is a problem
		Error_handler();
	}
}
void Error_handler(void)
{
	while(1);
}
