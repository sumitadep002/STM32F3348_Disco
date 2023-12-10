/*
 * stm32f334xx_gpio_driver.h
 *
 *  Created on: Apr 18, 2023
 *      Author: adeps
 */

#ifndef INC_STM32F334XX_GPIO_DRIVER_H_
#define INC_STM32F334XX_GPIO_DRIVER_H_

#include "stm32f334xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;				//possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t ;
typedef struct
{
	GPIO_RegDef_t *pGPIOx ;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11 	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15
/*
 * @GPIO_PIN_MODES
 *	GPIO pin possible modes
 */
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6


//GPIO pin possible otypes
#define GPIO_OP_TYPE_PP 0	//Push Pull
#define GPIO_OP_TYPE_OD 1	//Output Drain

//GPIO possible output speeds
#define GPIO_SPEED_LOW    0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST   2
#define GPIO_SPEED_HIGH   3

//GPIO pin pull up and pull down configuration macros
//pupd
#define GPIO_NO_PUPD 0	//pull up and pull down
#define GPIO_PIN_PU  1	//pull up
#define GPIO_PIN_PD  2	//pull down

//peripheral clock setup

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
//GPIO Initialization and DeInitialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//GPIO read write and toggle
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//GPIO IRQ configuration
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t PinNumber);



#endif /* INC_STM32F334XX_GPIO_DRIVER_H_ */
