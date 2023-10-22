/*
 * 007main.c
 *
 *  Created on: Jun 4, 2020
 *      Author: Nabli Hatem
 */

/*
 * SCL PB8
 * SDA PB9
 */

#include "stm32f303xx.h"
#include <string.h>
#include <stdio.h>
#define SlaveAddr   0x68
I2C_Handle_t I2C1_Handle;
char some_data[] = "We are testing I2C";

void delay()
{
	for(uint32_t i=0;i<50000/2;i++);
}
void I2C1_GPIOInit(void)
{
	GPIO_Handle_t  SPIGpioB;

	memset(&SPIGpioB,0,sizeof(SPIGpioB));


	SPIGpioB.pGPIOx = GPIOB;
	SPIGpioB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIGpioB.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	SPIGpioB.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	SPIGpioB.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PIN_PU;
	SPIGpioB.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;

	//SCL
	SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&SPIGpioB);

	//SDA
	SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIGpioB);
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

void I2C_Inits(void)
{
	I2C1_Handle.pI2Cx = I2C1;
	I2C1_Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM100;
	I2C1_Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;

	I2C_Init(&I2C1_Handle);

}

int main(void)
{
	//I2C pin init
	I2C1_GPIOInit();
	//I2C init
	I2C_Inits();

	I2C_MasterSendData(&I2C1_Handle, (uint8_t*)some_data, strlen(some_data), SlaveAddr);
	return 0;

}
