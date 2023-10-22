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

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIGpioA, SPIGpioB;
	memset(&SPIGpioA,0,sizeof(SPIGpioA));
	memset(&SPIGpioB,0,sizeof(SPIGpioB));
	SPIGpioA.pGPIOx = GPIOA;
	SPIGpioA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIGpioA.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_TYPE_PP;
	SPIGpioA.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIGpioA.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
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
	//SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIGpioB);

	//SCLK
	SPIGpioB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIGpioB);
}
void SPI2_Init(void)
{

	SPI_Handle_t SPI2handle;
	memset(&SPI2handle,0,sizeof(SPI2handle));
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_ClkSpeed = SPI_CLK_SPEED_DIV2;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DS_8BITS;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN; //software slave managment enabled for NSS pin
	SPI2handle.SPI_Config.SPI_SPE = SPI_SPE_EN;
	SPI_Init(&SPI2handle);


}
int main(void)
{
	char user_data[]="Hello";
	SPI_PeriClockControl(SPI2, ENABLE);

	SPI2_GPIOInit();
	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);
	SPI2_Init();

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	//lets confirm flag is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	//Disable the SPI2 peripheral
	SPI_PeriClockControl(SPI2, DISABLE);
	while(1);
	return 0;
}
