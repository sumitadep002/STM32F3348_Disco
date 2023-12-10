/*
 * stm32f334xx_rcc_driver.h
 *
 *  Created on: Apr 28, 2023
 *      Author: adeps
 */

#ifndef INC_STM32F334XX_RCC_DRIVER_H_
#define INC_STM32F334XX_RCC_DRIVER_H_

#include "stm32f334xx.h"


#define HSI 0 //internal 8MHz
#define HSE 1 //external 4MHz to 8MHz
#define PLL 2 //min = 2 x source to max = 16 source
uint32_t RCC_GetHCLKValue(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F334XX_RCC_DRIVER_H_ */
