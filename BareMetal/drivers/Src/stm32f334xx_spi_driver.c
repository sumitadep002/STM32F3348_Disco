/*
 * stm32f334xx_spi_driver.c
 *
 *  Created on: 20-Apr-2023
 *      Author: adeps
 */

#include "stm32f334xx.h"
uint8_t DR_DATA;

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


//SPI Initialization and DeInitialization
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//spi peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//configuring SPI_CR1_REGISTER
	uint32_t tempreg = 0;

	//device mode master/slave
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configure theFD/HD RX ONLY
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared to use SPI in 2 line unidirectional mode
		tempreg &= ~(1<<SPI_CR1_BIDIOE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set to use SPI in 1 line bidirectional HD mode
		tempreg |= (1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
 	{
		//BIDI mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit should be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	//configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR; //setting BR[2:0]


	//configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//configure the CHPA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//configuring the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

	//configure the DFF OR DS in CR2
	pSPIHandle->pSPIx->CR2 |= pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	SPI1_REG_RESET();
	SPI1_PCLK_DI();

}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
				SPI1_PCLK_EN();
		}
			else
			{
				SPI1_PCLK_DI();
			}

}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * data send and receive
 */

//Polling based code: Blocking type function call
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	//1. wait until TXE is set: until all the bits are transferred
	while(Len>0)
	{
		//wait if TXE is not set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == 0);


		//2. check the DS bit in CR2
		if((pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0f00)
		{
			//16. bit DS
			//1.  load the data  in  to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		//check SPI_CR2_DS for 8 or 16 bit data frame
		else if((pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0700)
		{
			//8 bit DS
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			pTxBuffer++;

		}
	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	//1. wait until TXE is set: until all the bits are transferred
		while(Len>0)
		{
			//wait if Receiver buffer in not empty
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

			//2. check the DS bit in CR1 for data format
			if((pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0xf00)
			{
				//16. bit DS
				//1.  load the data  from DR to RxBuffer
				 *((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else if((pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0700)
			{
				//8 bit DS
				pSPIx->DR = *((uint16_t*)pRxBuffer);

				Len--;
				pRxBuffer++;
			}
		}

}

uint8_t SPI_SendDataIt(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint8_t Len)
{
	uint8_t state = pSPIHandle-> TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	//1. save the tx buffer address and len information in some global variable
	pSPIHandle->pTxBuffer = pTxBuffer ;
	pSPIHandle->TxLen = Len;

	//2. mark the SPI state as busy in transmission so that not other code can take over
	//same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	//3. Enable the TXEIE control the bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	//4. Data Transmission will be handled by the ISR code (will implant later)
	}
	return state;
}
uint8_t SPI_ReceiveDataIt(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint8_t Len)
{
	uint8_t state = pSPIHandle-> RxState;
	if(state != SPI_BUSY_IN_RX)
	{
	//1. save the tx buffer address and len information in some global variable
	pSPIHandle->pRxBuffer = pRxBuffer ;
	pSPIHandle->RxLen = Len;

	//2. mark the SPI state as busy in transmission so that not other code can take over
	//same SPI peripheral until transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;
	//3. Enable the RXEIE control the bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	//4. Data Transmission will be handled by the ISR code (will implant later)
	}
	return state;
}

/*
 * spi peripheral control
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

//SS
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 *IRQ configuration and ISR Handling
 *
 */
//SPI IRQ configuration
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1,temp2;
	//check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//handler for TXEd
		spi_txe_interrupt_handle(pSPIHandle);
	}
	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}
	//check for ovr flag
	temp1 = pSPIHandle-> pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pSPIHandle-> pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount);
}

/*
 * helper functions
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
			//2. check the DS bit in CR2
			if((pSPIHandle->pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0f00)
			{
				//16. bit DS
				//1.  load the data  in  to the DR
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			//check SPI_CR2_DS for 8 or 16 bit data frame
			else if((pSPIHandle->pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0700)
			{
				//8 bit DS
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			if(!pSPIHandle->TxLen)
			{
				//TxLen is zero, so close the spi transmission and inform the application that
				//tx is over
				//this prevents interrupts from setting of TXE flag
				SPI_CloseTransmission(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
			//2. check the DS bit in CR2
			if((pSPIHandle->pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0f00)
			{
				//16. bit DS
				//1.  load the data from the DR
				(*(uint16_t*)pSPIHandle->pTxBuffer)=pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
			//check SPI_CR2_DS for 8 or 16 bit data frame
			else if((pSPIHandle->pSPIx->CR2 & ( 0xf << SPI_CR2_DS)) == 0x0700)
			{
				//8 bit DS
				*pSPIHandle->pTxBuffer=(uint8_t)pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
			if(!pSPIHandle->RxLen)
			{
				//TxLen is zero, so close the spi transmission and inform the application that
				//tx is over
				//this prevents interrupts from setting of TXE flag
				SPI_CloseReception(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
			}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//1.clear the ovr flag
	if(pSPIHandle->TxState!=SPI_BUSY_IN_TX)
	{
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer=NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//this is the weak implementation the applicaiton may overwrite this function
}
