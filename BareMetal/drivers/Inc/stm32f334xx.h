/*
 * stm32f334xx.h
 *
 *  Created on: Apr 18, 2023
 *      Author: adeps
 */

#ifndef INC_STM32F334XX_H_
#define INC_STM32F334XX_H_

#include <stdint.h>
#include <stddef.h>
#define __vo volatile

#define __weak __attribute__((weak))

/******************************************Processor Specific Details STARTS*******************************/
//NVIC base address
#define NVIC_BASE_ADDR 0xE0000000U

#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

/******************************************Processor Specific Details ENDS*******************************/

//some generic macros
#define ENABLE  	1
#define DISABLE 	0
#define SET     	ENABLE
#define RESET    	DISABLE
#define FLAG_RESET 	RESET
#define FLAG_SET 	SET

//IRQ numbers
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2_TS 	2
#define IRQ_NO_EXTI3		16
#define IRQ_NO_EXTI4		17
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
/***************************************************************************Addresses STARTS*********************************************************************/

//BASE ADDRESSES OF SRAM, FLASH AND SYSTEM MEMORY
#define FLASH_BASE_ADDR      0x08000000U //THE BASE ADDRESS OF FLASH MEMORY
#define SRAM_BASE_ADDR       0x20000000U //THE BASE ADDRESS OF SRAM
#define ROM_BASE_ADDR        0x1FFFD800U //THE BASE ADDRESS OF SYSTEM MEMORY OR ROM

//BASE ADDRESSES OF BUSES
#define PERIPH_BASE_ADDR    0x40000000U //THE BASE ADDRESS OF PERIPHERALS
#define APB1_BASE_ADDR PERIPH_BASE_ADDR //THE BASE ADDRESS OF APB1 PERIPHERALS
#define APB2_BASE_ADDR      0x40010000U //THE BASE ADDRESS OF APB2 PERIPHERALS
#define AHB1_BASE_ADDR      0x40020000U //THE BASE ADDRESS OF AHB1 PERIPHERALS
#define AHB2_BASE_ADDR      0x48000000U //THE BASE ADDRESS OF AHB2 PERIPHERALS
#define AHB3_BASE_ADDR      0x50000000U //THE BASE ADDRESS OF AHB3 PERIPHERALS



/*BASE ADDRESSES OF APB1 PERIPHERALS*/
#define TIM2_BASE_ADDR    (APB1_BASE_ADDR + 0x0000)
#define TIM3_BASE_ADDR    (APB1_BASE_ADDR + 0x0400)
#define TIM6_BASE_ADDR    (APB1_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR    (APB1_BASE_ADDR + 0x1400)
#define RTC_BASE_ADDR     (APB1_BASE_ADDR + 0x2800)
#define WWDG_BASE_ADDR    (APB1_BASE_ADDR + 0x2C00)
#define IWDG_BASE_ADDR    (APB1_BASE_ADDR + 0x3000)
#define PWR_BASE_ADDR     (APB1_BASE_ADDR + 0x7000)
#define DAC1_BASE_ADDR    (APB1_BASE_ADDR + 0x7400)
#define DAC2_BASE_ADDR    (APB1_BASE_ADDR + 0x9800)
#define I2C1_BASE_ADDR    (APB1_BASE_ADDR + 0x5400)
#define USART2_BASE_ADDR  (APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR  (APB1_BASE_ADDR + 0x4800)
#define bxCAN_BASE_ADDR   (APB1_BASE_ADDR + 0x6400)

/*BASE ADDRESSES OF APB2 PERIPHERALS*/
#define EXTI_BASE_ADDR     (APB2_BASE_ADDR + 0x0400)
#define TIM1_BASE_ADDR     (APB2_BASE_ADDR + 0x2C00)
#define SPI1_BASE_ADDR     (APB2_BASE_ADDR + 0x3000)
#define USART1_BASE_ADDR   (APB2_BASE_ADDR + 0x3800)
#define TIM15_BASE_ADDR    (APB2_BASE_ADDR + 0x4000)
#define TIM16_BASE_ADDR    (APB2_BASE_ADDR + 0x4400)
#define TIM17_BASE_ADDR    (APB2_BASE_ADDR + 0x4800)

//BASE ADDRESS OF AHB1 PERIPHERAL
#define RCC_BASE_ADDR   (AHB1_BASE_ADDR + 0x1000)

//BASE ADDRESSES OF AHB2
#define GPIOA_BASE_ADDR (AHB2_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR (AHB2_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR (AHB2_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR (AHB2_BASE_ADDR + 0x0C00)
#define GPIOF_BASE_ADDR (AHB2_BASE_ADDR + 0x1400)

#define SYSCFGEN_BASE_ADDR (APB2_BASE_ADDR + 0x0000)

/******************************************************************Addresses END************************************************/



/******************************************************************EXTI STARTS**************************************************/
#define EXTI ((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/*EXTERNAL interrupt specific structure*/
typedef struct
{
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
}EXTI_RegDef_t;

/*EXTI REGISTER BIT FIELD MACRO*/



/******************************************************************EXTI ENDS****************************************************/

/******************************************************************RCC STARTS***************************************************/
/*RCC specific structure*/


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

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)



/*
 * RCC_Get starts from here
 */

 //get HCLK



/*
 * RCC_Get starts end here
 */


/*****************************************************************************RCC ENDS******************************************************************/


//GPIO specific macros
#define GPIO_PIN_SET     ENABLE
#define GPIO_PIN_RESET    DISABLE

/*****************************************************************************SYSCFGEN STARTS******************************************************************/

#define SYSCFGEN	((SYSCFGEN_RegDef_t*)SYSCFGEN_BASE_ADDR)


//system configuration CLOCK ENABLE
#define SYSCFGEN_PCLK_EN()	(RCC->APB2ENR |= (1<<0))

//system configuration CLOCK DISABLE
#define SYSCFGEN_PCLK_DI()	(RCC->APB2ENR &= ~(1<<0)

//SYSCFGEN REGISTER DEFINITION
typedef struct
{
	__vo uint32_t CFGR[3];
	__vo uint32_t RCR;
	__vo uint32_t EXTICR[4];
}SYSCFGEN_RegDef_t;

/*****************************************************************************SYSCFGEN SARTS******************************************************************/



/*SPI specific structure*/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
}SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t*) SPI1_BASE_ADDR)

//SPI CLOCK ENABLE
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))

//SPI CLOCK DISABLE
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1<<12))

//SPI REGISTER RESET
#define SPI1_REG_RESET()  do { (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &=~(1<<12)); }while(0)




/*******************************************************************************************SPI ENDS*************************************************************/


/*******************************************************************************************GPIO STARTS**********************************************************/


//defining GPIO
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)

/*GPIO register specific macros */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;
}GPIO_RegDef_t;



//GBIO CLOCK ENABLE
#define GPIOA_PCLK_EN() (RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN() (RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN() (RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN() (RCC->AHBENR |= (1<<20))
#define GPIOF_PCLK_EN() (RCC->AHBENR |= (1<<22))

/*
 *clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI() (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI() (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI() (RCC->AHBENR &= ~(1<<20))
#define GPIOF_PCLK_DI() (RCC->AHBENR &= ~(1<<22))

//register reset

#define GPIOA_REG_RESET()  do { (RCC->AHBRSTR |= (1<<17)); (RCC->AHBRSTR &=~(1<<17)); }while(0)
#define GPIOB_REG_RESET()  do { (RCC->AHBRSTR |= (1<<18)); (RCC->AHBRSTR &=~(1<<18)); }while(0)
#define GPIOC_REG_RESET()  do { (RCC->AHBRSTR |= (1<<19)); (RCC->AHBRSTR &=~(1<<19)); }while(0)
#define GPIOD_REG_RESET()  do { (RCC->AHBRSTR |= (1<<20)); (RCC->AHBRSTR &=~(1<<20)); }while(0)
#define GPIOF_REG_RESET()  do { (RCC->AHBRSTR |= (1<<22)); (RCC->AHBRSTR &=~(1<<22)); }while(0)




/*******************************************************************************************GPIO ENDS**********************************************************/


/*******************************************************************************************USART STARTS**********************************************************/
#define USART1 ((USART_RegDef_t*) USART1_BASE_ADDR)
#define USART2 ((USART_RegDef_t*) USART2_BASE_ADDR)
#define USART3 ((USART_RegDef_t*) USART3_BASE_ADDR)

//USART CLOCK ENABLE
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))

//USART CLOCK DISABLE
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))

//USART register reset
#define USART1_REG_RESET()  do { (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &=~(1<<14)); }while(0)
#define USART2_REG_RESET()  do { (RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &=~(1<<17)); }while(0)
#define USART3_REG_RESET()  do { (RCC->APB1RSTR |= (1<<18)); (RCC->APB1RSTR &=~(1<<18)); }while(0)

/*
 * USART register definition macros
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t BRR;
	__vo uint32_t GTPR;
	__vo uint32_t RTOR;
	__vo uint32_t  RQR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t RDR;
	__vo uint32_t TDR;

}USART_RegDef_t;



/*******************************************************************************************USART ENDS**********************************************************/

/*******************************************************************************************I2C STARTS**********************************************************/
//I2C peripheral register definition structure

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t I2C_TIMINGR;
	__vo uint32_t TIMINGOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;
}I2C_RegDef_t;


//BIT POSITION definition macros I2C_CR1
#define I2C_CR1_PE 			0
#define I2C_CR1_TXIE 		1
#define I2C_CR1_ RXIE		2
#define I2C_CR1_ADDRIE		3
#define I2C_CR1_NACKIE		4
#define I2C_CR1_STOPIE		5
#define I2C_CR1_TCIE		6
#define I2C_CR1_ERRIE		7
#define I2C_CR1_DNF			8
#define I2C_CR1_ANF			12
#define I2C_CR1_TXDMAEN		14
#define I2C_CR1_RXDMAEN		15
#define I2C_CR1_SBC			16
#define I2C_CR1_NOSTRETCH	17
#define I2C_CR1_WUPEN		18
#define I2C_CR1_GCEN		19
#define I2C_CR1_SMBHEN		20
#define I2C_CR1_SMBDEN		21
#define I2C_CR1_ALERTEN		22
#define I2C_CR1_PCEN		23

//BIT POSITION definition macros I2C_CR2
#define I2C_CR2_SADD	0
#define I2C_CR2_RD_WRN	10
#define I2C_CR2_ADD10	11
#define I2C_CR2_HEAD10R	12
#define I2C_CR2_START	13
#define I2C_CR2_STOP	14
#define I2C_CR2_NACK 	15
#define I2C_CR2_NBYTES	16
#define I2C_CR2_RELOAD	24
#define I2C_CR2_AUTOEND	25
#define I2C_CR2_PECBYTE	26

//BIT POSITION definition macros I2C_OAR1
#define I2C_OAR1_OA1 		0
#define I2C_OAR1_OA1MODE	10
#define I2C_OAR1_OA1EN 		15

//BIT POSITION definition macros I2C_OAR2
#define I2C_OAR2_OA 	0
#define I2C_OAR2OA2MSK	8
#define I2C_OAR2OA2EN	15

//BIT POSITION definition macros I2C_TIMINGR
#define I2C_TIMINGR_SCLL 	0
#define I2C_TIMINGR_SCLH 	8
#define I2C_TIMINGR_SDADEL	16
#define I2C_TIMINGR_SCLDEL	20
#define I2C_TIMINGR_PRESC	28

//BIT POSITION definition macros I2C_TIMEOUTR
#define I2C_TIMEOUTR_TIMEOUTA 	0
#define I2C_TIMEOUTR_TIDLE		12
#define I2C_TIMEOUTR_TIMEOUTEN	15
#define I2C_TIMEOUTR_TIMOUTB	16
#define I2C_TIMEOUTR_TEXTEN		31

//BIT POSITION definition macros I2C_ISR
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
#define I2C_ISR_BUSY		15
#define I2C_ISR_DIR			16
#define I2C_ISR_ADDCODES 	17

//BIT POSITION definition macros I2C_ICR
#define I2C_ICR_ADDRCF
#define I2C_ICR_NACKCF
#define I2C_ICR_STOPCF
#define I2C_ICR_BERRCF
#define I2C_ICR_ARLOCF
#define I2C_ICR_OVRCF
#define I2C_ICR_PECCF
#define I2C_ICR_TIMOUTCF
#define I2C_ICR_ALERTCF

//BIT POSITION definition macros I2C_PECR
#define I2C_PECR_PEC 0

//BIT POSITION definition macros I2C_RXDR
#define I2C_RXDR_RXDATA 0

//BIT POSITION definition macros I2C_TXDR
#define I2C_PECR_TXDR 0


//I2C CLOCK ENABLE
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1<<21))

//I2C CLOCK DISABLE
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1<<21))

//I2C REGISTER RESET
#define I2C1_REG_RESET()  do { (RCC->APB1ENR |= (1<<21)); (RCC->APB1RSTR &=~(1<<21)); }while(0)

/*******************************************************************************************I2C ENDS**********************************************************/

/*******************************************************************************************CAN STARTS**********************************************************/
//CAN CLCOK ENABLE
#define CAN_PCLK_EN() (RCC->APB1ENR |= (1<<25))

//CAN CLCOK DISABLE
#define CAN_PCLK_DI()		(RCC->APB1ENR &= ~(1<<25))

//CAN register reset
#define CAN_REG_RESET()  do { (RCC->APB1RSTR |= (1<<25)); (RCC->APB1RSTR &=~(1<<25)); }while(0)

/*******************************************************************************************CAN ENDS**********************************************************/

/*******************************************************************************************TIMER STARTS**********************************************************/

//TIMERS CLOCK ENABLE
#define TIM1_PCLK_EN() (RCC->APB2ENR  |= (1<<11))
#define TIM2_PCLK_EN() (RCC->APB1ENR  |= (1<<0))
#define TIM3_PCLK_EN() (RCC->APB1ENR  |= (1<<1))
#define TIM6_PCLK_EN() (RCC->APB1ENR  |= (1<<4))
#define TIM7_PCLK_EN() (RCC->APB1ENR  |= (1<<5))
#define TIM15_PCLK_EN() (RCC->APB2ENR |= (1<<16))
#define TIM16_PCLK_EN() (RCC->APB2ENR |= (1<<17))
#define TIM17_PCLK_EN() (RCC->APB2ENR |= (1<<18))

//TIMERS CLOCK DISABLE
#define TIM1_PCLK_DI() (RCC->APB2ENR  &= ~(1<<11))
#define TIM2_PCLK_DI() (RCC->APB1ENR  &= ~(1<<0))
#define TIM3_PCLK_DI() (RCC->APB1ENR  &= ~(1<<1))
#define TIM6_PCLK_DI() (RCC->APB1ENR  &= ~(1<<4))
#define TIM7_PCLK_DI() (RCC->APB1ENR  &= ~(1<<5))
#define TIM15_PCLK_DI() (RCC->APB2ENR &= ~(1<<16))
#define TIM16_PCLK_DI() (RCC->APB2ENR &= ~(1<<17))
#define TIM17_PCLK_DI() (RCC->APB2ENR &= ~(1<<18))

//TIMER register reset
#define TIM1_REG_RESET()   do { (RCC->APB2RSTR |= (1<<11)); (RCC->APB2RSTR &=~(1<<11)); }while(0)
#define TIM2_REG_RESET()   do { (RCC->APB2RSTR |= (1<<0)); (RCC->APB2RSTR &=~(1<<0)); }while(0)
#define TIM3_REG_RESET()   do { (RCC->APB2RSTR |= (1<<1)); (RCC->APB2RSTR &=~(1<<1)); }while(0)
#define TIM6_REG_RESET()   do { (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &=~(1<<4)); }while(0)
#define TIM7_REG_RESET()   do { (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &=~(1<<5)); }while(0)
#define TIM15_REG_RESET()  do { (RCC->APB2RSTR |= (1<<16)); (RCC->APB2RSTR &=~(1<<16)); }while(0)
#define TIM16_REG_RESET()  do { (RCC->APB2RSTR |= (1<<17)); (RCC->APB2RSTR &=~(1<<17)); }while(0)
#define TIM17_REG_RESET()  do { (RCC->APB2RSTR |= (1<<18)); (RCC->APB2RSTR &=~(1<<18)); }while(0)

/*******************************************************************************************TIMER ENDS**********************************************************/


#include "stm32f334xx_gpio_driver.h"
#include "stm32f334xx_spi_driver.h"
#include "stm32f334xx_i2c_driver.h"
#include "stm32f334xx_usart_driver.h"
#include "stm32f334xx_rcc_driver.h"
#endif /* INC_STM32F334XX_H_ */
