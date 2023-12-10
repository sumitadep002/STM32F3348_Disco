/*
 * stm32f334xx_rcc_driver.c
 *
 *  Created on: May 3, 2023
 *      Author: adeps
 */

/*
 * NOTE: HSI internal oscillator 8MHz and HSE external oscillator 4MHz t o 32 MHz
 */

#include "stm32f334xx.h"



uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetHCLKValue(void)
{

	uint8_t divider=0;
	uint16_t ahbp;
	uint8_t clksrc;
	uint32_t SystemClk,hclk;
	clksrc = ((RCC->CFGR >> 2) & 0x3);
	//to configure source of frequency
	//0 means HSI internal RC = 8MHz and HSE external RC = 8MHz


	/*
	 * check the which oscillator being used at this time
	 * .
	 */
	if(clksrc == HSI || clksrc == HSE)
	{
		//HSI internal CLOCK
		SystemClk = 8000000;
	}
	/*else if(clksrc == 2)
	{
		//PLL as source of frequency
		SystemClk = RCC_getPLLOutputClock();
	}*/

	//for AHB
	//getting the value of clock frequency from 8 to 15
	divider = (RCC->CFGR >> 4) & 0xf;
	if(divider<8)
	{
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[divider-8];
	}

	hclk = (uint32_t)(SystemClk / ahbp);

	return hclk;
}

//get PCLK1
uint32_t RCC_GetPCLK1Value(void)
{
	uint8_t divider=0;
	uint32_t pclk1;
	uint16_t apb1p,ahbp;
	uint8_t clksrc;
	uint32_t SystemClk;
	clksrc = ((RCC->CFGR >> 4) & 0x7);
	//to configure source of frequency
	if(clksrc == 0 || clksrc == 1)
	{
		//HSI internal CLOCK
		SystemClk = 8000000;
	}
	/*else if(clksrc == 1)
	{
		//HSE external source of frequency
		SystemClk = 8000000;
	}
	else if(clksrc == 2)
	{
		//PLL as source of frequency
		SysClk = RCC_getPLLOutputClock();
	}*/

	//for AHB
	divider = (RCC->CFGR >> 8) & 0x7;
	if(divider<8)
	{
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[divider-8];
	}

//	for APB1
	divider = (RCC->CFGR >> 8) & 0x7;
	if(divider<4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p  = APB_PreScaler[divider-4];
	}
	pclk1 = (SystemClk / ahbp)/apb1p;

	pclk1 = (SystemClk / ahbp)/apb1p;

	return pclk1;
}

//get PCLK2
uint32_t RCC_GetPCLK2Value(void)
{
		uint8_t divider=0;
		uint32_t pclk2;
		uint16_t apb2p,ahbp;
		uint8_t clksrc;
		uint32_t SystemClk;
		clksrc = ((RCC->CFGR >> 4) & 0x7);
		//to configure source of frequency
		if(clksrc == 0 || clksrc == 1)
		{
			//HSI internal CLOCK
			SystemClk = 8000000;
		}
		/*else if(clksrc == 1)
		{
			//HSE external source of frequency
			SystemClk = 8000000;
		}
		else if(clksrc == 2)
		{
			//PLL as source of frequency
			SysClk = RCC_getPLLOutputClock();
		}*/

		//for AHB
		divider = (RCC->CFGR >> 8) & 0x7;
		if(divider<8)
		{
			ahbp = 1;
		}
		else{
			ahbp = AHB_PreScaler[divider-8];
		}

	//	for APB1
		divider = (RCC->CFGR >> 8) & 0x7;
		if(divider<4)
		{
			apb2p = 1;
		}
		else
		{
			apb2p  = APB_PreScaler[divider-4];
		}
		pclk2 = (SystemClk / ahbp)/apb2p;

		pclk2 = (SystemClk / ahbp)/apb2p;

		return pclk2;
}
