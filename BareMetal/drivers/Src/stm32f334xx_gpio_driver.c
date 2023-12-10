/*
 * stm32f334xx_gpio_driver.c
 *
 *  Created on: Apr 18, 2023
 *      Author: adeps
 */


#include "stm32f334xx.h"



void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
	if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
	}
}



uint8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *x)
{
	if(x == GPIOA )
	{
		return 0;
	}
	else if(x == GPIOB)
	{
		return 1;
	}
	else if(x == GPIOC)
	{
		return 2;
	}
	else if(x == GPIOD)
	{
		return 3;
	}
	else
	{
		return 4;
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp; //temp. register

	//Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);


	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non Interrupt mode
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		//enabling alternate function of gpio
		pGPIOHandle->pGPIOx->MODER &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
		pGPIOHandle->pGPIOx->MODER |= temp; //setting bit
	}
	else
	{
				//configure alternate function register
				uint8_t temp1,temp2;

				temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
				temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
				pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF << (4 * temp2)); //clearing bit
				pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));






		//the interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//1. configure the FTSR
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//set corresponding RTSR bit
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR bit
			EXTI->FTSR1 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{

			//1. configure the FTSR
			EXTI->FTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the gpio port selection in SYSCFGEN_EXTICR register
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFGEN_PCLK_EN();
		SYSCFGEN->EXTICR[temp1] = portcode << (temp2*4);
		//3. enable the exti interrupt deliver using IMR
		EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//3. configure the pupd setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &=~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure alternate function register
		uint32_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF << (4 * temp2)); //clearing bit
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));


	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
}


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) pGPIOx->IDR >> (PinNumber) & 0x00000001;
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to pin
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
//	value = (uint8_t) pGPIOx->IDR >> (PinNumber) & 0x00000001;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
		pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)

{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//program ISER0 = Interrupt Set Enable Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber <64)
		{
			//program ISER0
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >=64 && IRQNumber <96)
		{
			//program ISER0
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			//program ICER0 = Interrupt Clear Enable Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber <64)
		{
			//program ICER1 = Interrupt Clear Enable Register
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >=64 && IRQNumber <96)
		{
			//program ISER0
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR1 & (1<<PinNumber))
	{
		//clear
		EXTI->PR1 |= ( 1<< PinNumber);
	}
}


//will set the priority of the interrupt occurence
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount);
}
