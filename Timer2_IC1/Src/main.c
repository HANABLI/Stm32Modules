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
#include <stdio.h>

void LSE_Config(void);
void Error_handler(void);
void Sysclk_Config(uint8_t freq);
void TIMER2_Init();
void UART2_Init(void);
void GPIO_Init(void);
uint32_t input_capture[2] = {0};
uint8_t count = 1;
uint8_t is_capture_done = FALSE;

TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;

int main(void)
{
	uint32_t capture_difference = 0;
	double timer2_cnt_freq = 0 ;
	double timer2_cnt_res = 0;
	double user_signal_time_period = 0;
	double user_signal_freq = 0;
	char msg[100];
	HAL_Init();
	Sysclk_Config(SYS_CLOCK_FREQ_48_MHZ);

	//GPIO_Init();
	TIMER2_Init();
	LSE_Config();
	UART2_Init();

	if(HAL_TIM_IC_Start_IT(&htimer2, TIM_CHANNEL_1)!= HAL_OK)
		{
			Error_handler();

		}
	while(1)
	{

		if(is_capture_done)
		{
			if(input_capture[1] > input_capture[0])
				capture_difference = input_capture[1]- input_capture[0];
			else
				capture_difference = (0XFFFFFFFF - input_capture[0])+ input_capture[1];

		timer2_cnt_freq = (HAL_RCC_GetPCLK1Freq()*2)-(htimer2.Init.Prescaler+1);
		timer2_cnt_res = 1/ timer2_cnt_freq;
		user_signal_time_period = capture_difference * timer2_cnt_res;
		user_signal_freq = 1/user_signal_time_period;
		sprintf(msg,"Frequency of the signal applied = %f\r\n",user_signal_freq);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
		is_capture_done = FALSE;
		}
	}

	return 0;
}

void Sysclk_Config(uint8_t freq)
{

	uint8_t latency = 0;
		RCC_OscInitTypeDef osc_init;
		RCC_ClkInitTypeDef clk_init;

		osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
		osc_init.HSIState = RCC_HSI_ON;
		osc_init.LSEState = RCC_LSE_ON;
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

void LSE_Config(void)
{
#if 0
	RCC_OscInitTypeDef Osc_Init;
	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	Osc_Init.LSEState = RCC_LSE_ON;
	if(HAL_RCC_OscConfig(&Osc_Init)!= HAL_OK)
	{
		Error_handler();
	}
#endif
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(!is_capture_done)
	{
		if(count == 1)
		{
			input_capture[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count ++;
		}
		else if(count == 2)
		{
			input_capture[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count = 1;
			is_capture_done = TRUE;
		}
	}
}
void TIMER2_Init(void)
{
	TIM_IC_InitTypeDef timer2IC_Config;
	htimer2.Instance = TIM2;
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP ;
	htimer2.Init.Period = 0xFFFFFFFF;
	htimer2.Init.Prescaler = 1;
	if(HAL_TIM_IC_Init(&htimer2)!= HAL_OK)
	{
		Error_handler();
	}
	timer2IC_Config.ICFilter = 0 ;
	timer2IC_Config.ICPolarity = TIM_ICPOLARITY_RISING;
	timer2IC_Config.ICPrescaler = TIM_ICPSC_DIV1;
	timer2IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if(HAL_TIM_IC_ConfigChannel(&htimer2, &timer2IC_Config, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_handler();
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
