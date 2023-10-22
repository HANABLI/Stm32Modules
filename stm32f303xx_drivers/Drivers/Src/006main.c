/*
 * 006main.c
 *
 *  Created on: May 29, 2020
 *      Author: Nabli Hatem
 */


/*
 * 004main.c
 *
 *  Created on: May 26, 2020
 *      Author: Nabli Hatem
 */
#include <string.h>
#include "stm32f303xx.h"

//SPI2_MOSI 		PA11 AF5
//SPI2_MISO			PA10 AF5
//SPI2_SCLK			PB13 AF5
//SPI2_NSS			PB12 AF5

//Command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					SET
#define LED_OFF					RESET

#define LED_PIN					9
//arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

void delay()
{
	for(uint32_t i=0;i<50000/2;i++);
}
void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIGpioA, SPIGpioB;
	memset(&SPIGpioA,0,sizeof(SPIGpioA));
	memset(&SPIGpioB,0,sizeof(SPIGpioB));
	SPIGpioA.pGPIOx = GPIOA;
	SPIGpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIGpioA.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	SPIGpioA.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIGpioA.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PIN_PU;
	SPIGpioA.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;

	//MISO
	//SPIGpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	//GPIO_Init(&SPIGpioA);

	//MOSI
	SPIGpioA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&SPIGpioA);

	SPIGpioB.pGPIOx = GPIOB;
	SPIGpioB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIGpioB.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	SPIGpioB.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIGpioB.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	SPIGpioB.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_HIGH;

	//NSS
	SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIGpioB);

	//SCLK
	SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIGpioB);
}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t ButtonTest;
	memset(&ButtonTest,0,sizeof(ButtonTest));
	ButtonTest.pGPIOx = GPIOC;
	ButtonTest.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	ButtonTest.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	ButtonTest.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_FAST;
	ButtonTest.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	GPIO_DeInit(GPIOC);

	GPIO_Init(&ButtonTest);
}
void SPI2_Init(void)
{

	SPI_Handle_t SPI2handle;
	memset(&SPI2handle,0,sizeof(SPI2handle));
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_ClkSpeed = SPI_CLK_SPEED_DIV8; //generate sclk of 2MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DS_8BITS;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //software slave managment enabled for NSS pin

	//when SSOE = 1 then :
	// when SSM = 0 , if SPE= 1 then NSS o/p = 0 automaticly
	// when SSM = 0 , if SPE = 0 then NSS o/p =1 automaticly
	SPI_Init(&SPI2handle);


}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;
}
int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read ;
	GPIO_ButtonInit();

	//enable the SPI2 peripheral
	SPI_PeriClockControl(SPI2, ENABLE);

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();
	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Init();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	/*
		 * making SSOE 1 does NSS output enable.
		 * The NSS pin is automatically managed by the hardware.
		 * i.e when SPE=1, NSS will be pulled to low
		 * and NSS pin will be high when SPE = 0
		 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay();

		//enable the SPI2 peripheral
		SPI_PeriControle(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>		<value(1)>
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		SPI_SendData(SPI2, &commandCode, 1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		//send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI2,&dummy_write , 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1]	= LED_ON;
			// sending
			SPI_SendData(SPI2, args, 2);
		}
		//lets confirm flag is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		//Disable the SPI2 peripheral
		SPI_PeriControle(SPI2, DISABLE);
	}
	return 0;
}
