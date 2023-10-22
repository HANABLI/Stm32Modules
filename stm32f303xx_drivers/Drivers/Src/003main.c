/*
 * 003main.c
 *
 *  Created on: May 19, 2020
 *      Author: Nabli Hatem
 */
#include <string.h>
#include "stm32f303xx.h"
void delay(){
	for(uint32_t i=0;i<5;i++){

	}
}

int main(void){
	GPIO_Handle_t LedTest, ButtonTest;
	memset(&LedTest,0,sizeof(LedTest));
	memset(&ButtonTest,0,sizeof(ButtonTest));
	LedTest.pGPIOx = GPIOA;
	LedTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	LedTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LedTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	LedTest.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	LedTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;

	GPIO_DeInit(GPIOA);
	GPIO_Init(&LedTest);



	ButtonTest.pGPIOx = GPIOC;
	ButtonTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ButtonTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	ButtonTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_FAST;
	ButtonTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;

	GPIO_DeInit(GPIOC);
	GPIO_Init(&ButtonTest);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, IRQ_PRIORITY_NB15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);


}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	//delay();
}
