/*
 * main.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Nabli Hatem
 */
#include "stm32f3xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#define SYS_CLOCK_FREQ_24_MHZ	24
#define SYS_CLOCK_FREQ_48_MHZ	48
#define SYS_CLOCK_FREQ_72_MHZ	72
#define TRUE	1
#define FALSE	0

void UART2_Init(void);
void Error_handler(void);
void Sysclk_Config_HSE(uint8_t freq);
UART_HandleTypeDef huart2;
char msg[100];
int main(void)
{
	HAL_Init();

	Sysclk_Config_HSE(SYS_CLOCK_FREQ_48_MHZ);

	UART2_Init();

	memset(msg,0,sizeof(msg));
	sprintf(msg,"SYSCLK : %ld\r\n",HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"AHBCLK : %ld\r\n",HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"APB1CLK : %ld\r\n",HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"APB2CLK : %ld\r\n",HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	while(1);

	return 0;
}

void Sysclk_Config_HSE(uint8_t freq)
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
