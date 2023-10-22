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

	if(HAL_TIM_PWM_Start(&htimer2, TIM_CHANNEL_1)!=HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_PWM_Start(&htimer2, TIM_CHANNEL_2)!=HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_PWM_Start(&htimer2, TIM_CHANNEL_3)!=HAL_OK)
	{
		Error_handler();
	}
	if(HAL_TIM_PWM_Start(&htimer2, TIM_CHANNEL_4)!=HAL_OK)
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
	//osc_init.HSICalibrationValue = 16;
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
	TIM_OC_InitTypeDef tim2PWM;
	htimer2.Instance = TIM2;
	htimer2.Init.Period = 10000-1;
	htimer2.Init.Prescaler = 4799;
	if(HAL_TIM_PWM_Init(&htimer2)!= HAL_OK)
	{
		Error_handler();
	}
	memset(&tim2PWM,0,sizeof(tim2PWM));
	tim2PWM.OCMode = TIM_OCMODE_PWM1;
	tim2PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim2PWM.Pulse = htimer2.Init.Period*25/100;
	if(HAL_TIM_PWM_ConfigChannel(&htimer2,&tim2PWM, TIM_CHANNEL_1)!= HAL_OK)
	{
		Error_handler();
	}
	tim2PWM.Pulse = htimer2.Init.Period*45/100;
	if(HAL_TIM_PWM_ConfigChannel(&htimer2,&tim2PWM, TIM_CHANNEL_2)!= HAL_OK)
	{
		Error_handler();
	}
	tim2PWM.Pulse = htimer2.Init.Period*75/100;
	if(HAL_TIM_PWM_ConfigChannel(&htimer2,&tim2PWM, TIM_CHANNEL_3)!= HAL_OK)
	{
		Error_handler();
	}
	tim2PWM.Pulse = htimer2.Init.Period*95/100;
	if(HAL_TIM_PWM_ConfigChannel(&htimer2,&tim2PWM, TIM_CHANNEL_4)!= HAL_OK)
	{
		Error_handler();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

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
