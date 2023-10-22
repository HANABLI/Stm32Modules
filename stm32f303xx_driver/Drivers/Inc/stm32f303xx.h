/*
 * stm32f303xx.h
 *
 *  Created on: May 15, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_
#include <stdint.h>
#define __vo volatile
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASE_ADDR		0x08000000U
#define SRAM_BASE_ADDR		0x20000000U
#define ROM_BASE_ADDR		0x1FFFD800U


/*
 * AHB and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE				0x40000000U
#define APB1_PERIPH_BASE_ADDR	PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR 	0x40010000U
#define AHB1_PERIPH_BASE_ADDR	0x40020000U
#define AHB2_PERIPH_BASE_ADDR	0x48000000U
#define AHB3_PERIPH_BASE_ADDR	0x50000000U
#define AHB4_PERIPH_BASE_ADDR	0x60000000U


/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define RCC_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x1000)

#define GPIOA_BASE_ADDR 		(AHB2_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR 		(AHB2_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR			(AHB2_PERIPH_BASE_ADDR + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C3_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x5400)
#define SPI2_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x3C00)
#define USART2_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR			(APB1_PERIPH_BASE_ADDR + 0x5000)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x3C00)
#define USART1_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x0400)
#define EXTI_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x0400)
#define SYSCFG_BASE_ADDR		(APB2_PERIPH_BASE_ADDR + 0x0000)


/************************peripheral register definition structures*******************/

/*
 * Note : Registers of a peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t MODER; /*!< GPIO port mode register , Address offset 0x00 */
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR;
	__vo uint32_t BRR;


}GPIO_RegDef_t;

/*
 * -------------------------------------------------------
 * peripheral register definition structure for RCC
 * -------------------------------------------------------
 */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
}RCC_RegDef_t;

/*
 * -------------------------------------------------------------------------------
 * peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t
 * -------------------------------------------------------------------------------
 */

#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define	RCC                 ((RCC_RegDef_t*)RCC_BASE_ADDR)


/*
 * -------------------------------------------------
 * Clock enable macros for GPIOx peripherals
 * -------------------------------------------------
 */

#define GPIOA_PCLK_EN() 		(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()			(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()			(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()			(RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()			(RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()			(RCC->AHBENR |= (1 << 22))
#define GPIOG_PCLK_EN()			(RCC->AHBENR |= (1 << 23))
#define GPIOH_PCLK_EN()			(RCC->AHBENR |= (1 << 16))

/*
 * --------------------------------------------------
 * Clock enable macros for I2Cx peripherals
 * --------------------------------------------------
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))

/*
 * --------------------------------------------------
 * Clock enable macros for SPIx peripherals
 * --------------------------------------------------
 */

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 15))

/*
 * --------------------------------------------------
 * Clock enable macros for USARTx peripherals
 * --------------------------------------------------
 */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))



/*
 * --------------------------------------------------
 * Clock enable macros for SYSCFG peripherals
 * --------------------------------------------------
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0))



/*
 * -------------------------------------------------
 * Clock disable macros for GPIOx peripherals
 * -------------------------------------------------
 */

#define GPIOA_PCLK_DI() 		(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()			(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()			(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()			(RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI()			(RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI()			(RCC->AHBENR &= ~(1 << 22))
#define GPIOG_PCLK_DI()			(RCC->AHBENR &= ~(1 << 23))
#define GPIOH_PCLK_DI()			(RCC->AHBENR &= ~(1 << 16))

/*
 * --------------------------------------------------
 * Clock disable macros for I2Cx peripherals
 * --------------------------------------------------
 */

#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))

/*
 * --------------------------------------------------
 * Clock disable macros for SPIx peripherals
 * --------------------------------------------------
 */

#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 15))

/*
 * --------------------------------------------------
 * Clock disable macros for USARTx peripherals
 * --------------------------------------------------
 */
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock disable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0))


/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{ (RCC->AHBENR |= (1 << 17));  (RCC->AHBENR  &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHBENR |= (1 << 18));  (RCC->AHBENR  &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHBENR |= (1 << 19));  (RCC->AHBENR  &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHBENR |= (1 << 20));  (RCC->AHBENR  &= ~(1 << 20));}while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHBENR |= (1 << 21));  (RCC->AHBENR  &= ~(1 << 21));}while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHBENR |= (1 << 22));  (RCC->AHBENR  &= ~(1 << 22));}while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHBENR |= (1 << 23));  (RCC->AHBENR  &= ~(1 << 23));}while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHBENR |= (1 << 16));  (RCC->AHBENR  &= ~(1 << 16));}while(0)

/*
 * Generic macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f303xx_gpio_driver.h"

#endif /* INC_STM32F303XX_H_ */
