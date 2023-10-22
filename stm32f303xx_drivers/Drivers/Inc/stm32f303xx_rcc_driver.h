/*
 * stm32f303xx_rcc_driver.h
 *
 *  Created on: Jun 8, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_RCC_DRIVER_H_
#define INC_STM32F303XX_RCC_DRIVER_H_
#include "stm32f303xx.h"
uint32_t RCC_GetPCLKValue(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock();
#endif /* INC_STM32F303XX_RCC_DRIVER_H_ */
