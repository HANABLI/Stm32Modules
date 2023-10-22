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
#define __weak   			__attribute__((weak))
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

/*
 * IRQ(Interrupt Request) Numbers of STM32F303xx MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define	IRQ_NO_I2C1_EV		31
#define	IRQ_NO_I2C1_ER		32

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0	((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1	((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2	((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3	((__vo uint32_t*)0xE000E10c)
#define NVIC_ISER4	((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5	((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6	((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7	((__vo uint32_t*)0xE000E11c)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 	((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1	((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2	((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 	((__vo uint32_t*)0xE000E18c)
#define NVIC_ICER4	((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5	((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6	((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7	((__vo uint32_t*)0xE000E19c)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED		4

/*
 * IRQ priority number
 */

#define IRQ_PRIORITY_NB0		0
#define IRQ_PRIORITY_NB1		1
#define IRQ_PRIORITY_NB2		2
#define IRQ_PRIORITY_NB3		3
#define IRQ_PRIORITY_NB4		4
#define IRQ_PRIORITY_NB5		5
#define IRQ_PRIORITY_NB6		6
#define IRQ_PRIORITY_NB7		7
#define IRQ_PRIORITY_NB8		8
#define IRQ_PRIORITY_NB9		9
#define IRQ_PRIORITY_NB10		10
#define IRQ_PRIORITY_NB11		11
#define IRQ_PRIORITY_NB12		12
#define IRQ_PRIORITY_NB13		13
#define IRQ_PRIORITY_NB14		14
#define IRQ_PRIORITY_NB15		15

/************************peripheral register definition structures*******************/

/*
 * Note : Registers of a peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t MODER ; /*!< GPIO port mode register , Address offset 0x00 */
	__vo uint32_t OTYPER ;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;


}GPIO_RegDef_t ;



/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;





/*
 *peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;

}I2C_RegDef_t;


/*
 * --------------------------------------------------------
 * peripheral register definition structure for USART
 * --------------------------------------------------------
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;
}USART_RegDef_t;
/*
 * --------------------------------------------------------
 * peripheral register definition structure for EXTI
 * --------------------------------------------------------
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * -------------------------------------------------------
 * peripheral register definition structure for SYSCFG
 * -------------------------------------------------------
 */
typedef struct
{
	__vo uint32_t CFGR1;
	__vo uint32_t RCR;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
	__vo uint32_t CFGR4;

}SYSCFG_RegDef_t;

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
#define EXTI				((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG    			((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1           		((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASE_ADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2				((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C3				((I2C_RegDef_t*)I2C1_BASE_ADDR)

#define USART1   			((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2				((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3				((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4				((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5				((USART_RegDef_t*)UART5_BASE_ADDR)
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
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 30))

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
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 30))

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

#define GPIOA_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 17));  (RCC->AHBRSTR  &= ~(1 << 17));}while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 18));  (RCC->AHBRSTR  &= ~(1 << 18));}while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 19));  (RCC->AHBRSTR  &= ~(1 << 19));}while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 20));  (RCC->AHBRSTR  &= ~(1 << 20));}while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 21));  (RCC->AHBRSTR  &= ~(1 << 21));}while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 22));  (RCC->AHBRSTR  &= ~(1 << 22));}while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 23));  (RCC->AHBRSTR  &= ~(1 << 23));}while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 16));  (RCC->AHBRSTR  &= ~(1 << 16));}while(0)

/*
 * Macros to reset SPIx peripherals
 */

#define SPI1_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15));}while(0)
/*
 * this macro return code between 0 to 7 for a given GPIO base address(x)
 *
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:\
									(x==GPIOG)?6:\
									(x==GPIOH)?7:0)

/*
 * @SPI_CR1_REG
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR          	2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_CRCL			11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * @SPI_CR2_REG
 */
#define SPI_CR2_SSOE			2
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXEIE			6
#define SPI_CR2_TXEIE			7
#define SPI_CR2_DS				8


/*
 * @SPI_SR_REG
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*
 * @I2C_CR1_REG
 */
#define I2C_CR1_PE				0
#define I2C_CR1_TXIE			1
#define I2C_CR1_RXIE			2
#define I2C_CR1_ADDRIE			3
#define I2C_CR1_NACKIE			4
#define I2C_CR1_STOPIE			5
#define I2C_CR1_TCIE			6
#define I2C_CR1_ERRIE			7
#define I2C_CR1_DNF				8
#define I2C_CR1_ANFOFF			12
#define I2C_CR1_TXDMAEN			14
#define I2C_CR1_RXDMAEN			15
#define I2C_CR1_SBC				16
#define I2C_CR1_NOSTRETCH		17
#define I2C_CR1_WUPEN			18
#define I2C_CR1_GOEN			19
#define I2C_CR1_SMBHEN			20
#define I2C_CR1_SMBDEN			21
#define I2C_CR1_ALERTEN			22
#define I2C_CR1_PECEN			23

/*
 * @I2C_CR2_REG
 */
#define I2C_CR2_SADD			0
#define I2C_CR2_RD_WRN			10
#define I2C_CR2_ADD10			11
#define I2C_CR2_HEAD10R			12
#define I2C_CR2_START			13
#define I2C_CR2_STOP			14
#define I2C_CR2_NACK			15
#define I2C_CR2_NBYTES			16
#define I2C_CR2_RELOAD			24
#define I2C_CR2_AUTOEND			25
#define I2C_CR2_PECBYTE			26
/*
 * @I2C_ICR_REG
 */
#define I2C_ICR_ADDRCF		3
#define I2C_ICR_NACKCF		4
#define I2C_ICR_STOPCF		5
#define I2C_ICR_BERRCF		8
#define I2C_ICR_ARLOCF		9
#define I2C_ICR_OVRCF		10
#define I2C_ICR_PECCF		11
#define I2C_ICR_TIMOUTCF		12
#define I2C_ICR_ALERTCF		13

/*
 * @I2C_ISR_REG
 */
#define I2C_ISR_TXE			0
#define I2C_ISR_TXIS		1
#define I2C_ISR_RXNE		2
#define I2C_ISR_ADDR		3
#define I2C_ISR_NACKF		4
#define I2C_ISR_STOPF		5
#define I2C_ISR_TC			6
#define I2C_ISR_TCR			7
#define I2C_ISR_BERR		8
#define I2C_ISR_ARLO		9
#define I2C_ISR_OVR			10
#define I2C_ISR_PECERR		11
#define I2C_ISR_TIMEOUT		12
#define I2C_ISR_ALERT		13
#define I2C_ISR_BUSSY		15
#define I2C_ISR_DIR			16
#define I2C_ISR_ADDCODE		17


#define USART_CR1_UE		0
#define USART_CR1_UESM		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M0		12
#define USART_CR1_MME		13
#define USART_CR1_CMIE		14
#define USART_CR1_OVER8		15
#define USART_CR1_DEDT		16
#define USART_CR1_DEAT		21
#define USART_CR1_RTOIE		26
#define USART_CR1_EOBIE		27
#define USART_CR1_M1		28

/*
 * @USART_CR2_Reg
 */
#define USART_CR2_ADDM7		4
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14
#define USART_CR2_SWAP		15
#define USART_CR2_RXINV		16
#define USART_CR2_TXINV		17
#define USART_CR2_DATAINV	18
#define USART_CR2_MSBFIRST	19
#define USART_CR2_ABREN		20
#define USART_CR2_ABRMOD	21
#define USART_CR2_RTOEN		22
#define USART_CR2_ADD0		24
#define USART_CR2_ADD1		28
/*
 * @USART_CR3_Reg
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11
#define USART_CR3_OVRDIS	12
#define USART_CR3_DDRE		13
#define USART_CR3_DEM		14
#define USART_CR3_DEP		15
#define USART_CR3_SCARCNT	17
#define USART_CR3_WUS		20
#define USART_CR3_WUFIE		22

/*
 * @USART_GTPR_Reg
 */

#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8

/*
 * @USART_RQR_Reg
 */
#define USART_RQR_ABRRQ		0
#define USART_RQR_SBKRQ		1
#define USART_RQR_MMRQ		2
#define USART_RQR_RXFRQ		3
#define USART_RQR_TXFRQ		4
/*
 * @USART_ISR_Reg
 */
#define USART_ISR_PE		0
#define USART_ISR_FE		1
#define USART_ISR_NF		2
#define USART_ISR_ORE		3
#define USART_ISR_IDLE		4
#define USART_ISR_RXNE		5
#define USART_ISR_TC		6
#define USART_ISR_TXE		7
#define USART_ISR_LBDF		8
#define USART_ISR_CTSIF		9
#define USART_ISR_CTS		10
#define USART_ISR_RTOF		11
#define USART_ISR_EOBF		12
#define USART_ISR_ABRE		14
#define USART_ISR_ABRF		15
#define USART_ISR_BUSY		16
#define USART_ISR_CMF		17
#define USART_ISR_SBKF		18
#define USART_ISR_RWU		19
#define USART_ISR_WUF		20
#define USART_ISR_TEACK		21
#define USART_ISR_REACK		22

/*
 * @USART_ICR_Reg
 */
#define USART_ICR_PECF		0
#define USART_ICR_FECF		1
#define USART_ICR_NCF		2
#define USART_ICR_ORECF		3
#define USART_ICR_IDLECF	4
#define USART_ICR_TCCF		6
#define USART_ICR_LBDCF		8
#define USART_ICR_CTSCF		9
#define USART_ICR_RTOCF		11
#define USART_ICR_EOBCF		12
#define USART_ICR_CMCF		17
#define USART_ICR_WUCF		20
/*
 * Generic macros
 */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET 		RESET
#define FLAG_SET		SET

#include "stm32f303xx_gpio_drivers.h"
#include "stm32f303xx_spi_driver.h"
#include "stm32f303xx_i2c_drivers.h"
#include "stm32f303xx_rcc_driver.h"
#include "stm32f303xx_uart_driver.h"
#endif /* INC_STM32F303XX_H_ */
