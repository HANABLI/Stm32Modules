/*
 * 008main.c
 *
 *  Created on: Jun 8, 2020
 *      Author: Nabli Hatem
 */


//PA2	USART_TX	AF7
//PA3	USART_RX	AF7
#include "stm32f303xx.h"
#include <string.h>
#include <stdio.h>
char some_data[1024] = "testing USART ";
USART_Handle_t pUSART_Handle;
void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}
void USART2_GPIOInit(void)
{
	GPIO_Handle_t  USARTGpioA;

	memset(&USARTGpioA,0,sizeof(USARTGpioA));


	USARTGpioA.pGPIOx = GPIOA;
	USARTGpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTGpioA.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_OD;
	USARTGpioA.GPIO_PinConfig.GPIO_PinAltFuncMode = 7;
	USARTGpioA.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PIN_PU;
	USARTGpioA.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;

	//USART_TX
	USARTGpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_DeInit(GPIOA);
	GPIO_Init(&USARTGpioA);

	//USART_RX
	//USARTGpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	//GPIO_Init(&USARTGpioA);
}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t ButtonTest;
	memset(&ButtonTest,0,sizeof(ButtonTest));
	ButtonTest.pGPIOx = GPIOC;
	ButtonTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ButtonTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	ButtonTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;
	ButtonTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	GPIO_DeInit(GPIOC);

	GPIO_Init(&ButtonTest);
}
void USART_Inits()
{


	pUSART_Handle.pUSARTx = USART2;
	pUSART_Handle.USART_Config.USART_ModeConf = USART_MODE_ONLY_TX;
	pUSART_Handle.USART_Config.USART_BaudConf = USART_STD_BAUD_115200;
	pUSART_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	pUSART_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	pUSART_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	pUSART_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART2_GPIOInit();
	USART_Init(&pUSART_Handle);
}
int main(void)
{
	USART_Inits();
	GPIO_ButtonInit();
	USART_PeriControle(USART2, ENABLE);
	while(1)
		{
			//wait till button is pressed
			while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
			delay();
			USART_SendData(&pUSART_Handle, (uint8_t*)some_data, strlen(some_data));

		}
	return 0;
}
