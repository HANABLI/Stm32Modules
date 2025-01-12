/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: May 16, 2020
 *      Author: Nabli Hatem
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include "stm32f303xx.h"


typedef struct
{
	uint32_t GPIO_PinNumber;
	uint32_t GPIO_PinMode;			/*!< possible mode values from @GPIO_PIN_MODES >*/
	uint32_t GPIO_pinSpeed;			/*!< possible speed level from @GPIO_SPEED_LEVEL >*/
	uint32_t GPIO_pinPuPdControl; 	/*!< pull-up or pull-down configuration from @GPIO_PU_PD_CONFIG >*/
	uint32_t GPIO_pinOPType;		/*!< Output type from @GPIO_PIN_OUTPUT_TYPES >*/
	uint32_t GPIO_PinAltFuncMode; 	/* !< Alternate function parameter from  @GPIO_ALTFUN_CONFIG>*/
}GPIO_PinConfig_t;

/*
 *
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx ;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;



/*
 * GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO possible MODE macros
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define	GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6
/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO possible output types macros
 */
#define GPIO_OP_TYPE_PP     0    /*push pull*/
#define GPIO_OP_TYPE_OD		1	/*Open drain*/


/*@GPIO_SPEED_LEVEL
 * GPIO possible output speed macros
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PU_PD_CONFIG
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO_ALTFUN_CONFIG
 * TODO macros
 */


/*****************************************************************************
 * 						APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *******************************************************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);
/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFrom_InputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t  Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
