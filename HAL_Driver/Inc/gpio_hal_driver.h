/*
 *  stm32f446xx_gpio_hal_driver.h
 *  Definition to control GPIO peripheral operation
 *  Created on: Oct 10, 2020
 *  Author: Chirag Jethava
 *  email:  ercjethava@gmail.com
 */

#ifndef HAL_DRIVER_INC_GPIO_HAL_DRIVER_H_
#define HAL_DRIVER_INC_GPIO_HAL_DRIVER_H_

// MCU specific header file
#include "stm32f446xx.h"

/*
 * GPIO pin Configuration structure
 */
typedef struct GPIO_PinConfig
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			
	uint8_t GPIO_PinSpeed;			
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 *  Handle structure for a GPIO pin
 */

typedef struct GPIO_Handle
{
	GPIO_RegDef_t *pGPIOx;       		    /* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;        /* This holds GPIO pin configuration settings */

}GPIO_Handle_t;


/*
 * Macro used for GPIO_PIN_NUMBERS
 * 
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * Macro used for GPIO Pin modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*
 * Macro used for GPIO pin output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * Macro used for GPIO pin output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * Macro used for GPIO pin pull up AND pull down configuration
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



 /**************	APIs supported by this driver ********************/

/*
 * GIPO Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Initialize and De-initialize
 */
void GPIO_Initialize(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInitialize(GPIO_RegDef_t *pGPIOx);


/*
 * GPIO Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * GPIO IRQ Configuration
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * GPIO ISR handling
 */
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* HAL_DRIVER_INC_GPIO_HAL_DRIVER_H_ */



