/*
 * stm32f334xx_spi_driver.h
 *
 *  Created on: 20-Apr-2023
 *      Author: adeps
 */

#ifndef INC_STM32F334XX_SPI_DRIVER_H_
#define INC_STM32F334XX_SPI_DRIVER_H_

#include "stm32f334xx.h"
#include "stddef.h"

/*
 * SPI application events
 */
#define SPI_EVENT_TX_CMPLT		0
#define SPI_EVENT_RX_CMPLT		1
#define SPI_EVENT_OVR_ERR		2
#define SPI_EVENT_CRC_ERR		4

//Possible SPI Application States
#define SPI_READY	0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

//Configuration structure for spi peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;	//device mode master/slave
	uint8_t SPI_BusConfig;	//FD/SD/SIMPLEX
	uint8_t SPI_SclkSpeed;	//clock speed
	uint8_t SPI_DS;			//frame format
	uint8_t SPI_CPOL;		//clock
	uint8_t SPI_CPHA;		//clock phase
	uint8_t SPI_SSM;		//software slave management
}SPI_Config_t;

/*
 * @SPI_DeviceModes
 */

#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DIFF
 */
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW 0
#define SPI_CPOL_HIGH 1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW  0
#define SPI_CPHA_HIGH 1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI 0
#define SPI_SSM_EN 1

//SPi related flag status definition
#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 	(1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)


//spi definition structure
typedef struct
{
	SPI_RegDef_t *pSPIx;	//to hold the base address of the SPIx  (SPI1)
	SPI_Config_t SPIConfig;	//
	uint8_t *pTxBuffer;		//Tx Buffer
	uint8_t *pRxBuffer;		//Rx Buffer
	uint32_t TxLen;			//to store lenth of message to be transmtted
	uint32_t RxLen;			//to store lenth of message to be received
	uint8_t TxState;		//to store the state of transmission process
	uint8_t RxState;		//to store the state of the reception process
}SPI_Handle_t;


/*****************************************************************************SPI REGISTERS BIT DEFINITIONS******************************************************************/

//SPI CR1 REGISTER BIT FIELD DEFINITION
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3  //BR[2:0]=5 4 3
#define SPI_CR1_SPE 		6	//SPI enable
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8	//
#define SPI_CR1_SSM 		9	//SSM enable/disable
#define SPI_CR1_RXONLY 		10	//RXONLY
#define SPI_CR1_CRCL 		11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13	//CRC enable/disable
#define SPI_CR1_BIDIOE 		14	//BIDI mode enable/disable
#define SPI_CR1_BIDIMODE 	15	//Configure BIDI mode

//SPI CR1 REGISTER BIT FIELD DEFINITION

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8 //DS[3:0] = 11 10 9 8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14

//SPI SR REGISTER  BIT FIELD DEFINITION
#define SPI_SR_RXNE 	 	0
#define SPI_SR_TXE			1	//gets set when transmit buffer gets empty
#define SPI_SR_CRCERR		4	//gets set
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9  //FRLVL[1:0]= 10 9
#define SPI_SR_FTLVL		11 //FTLVL[1:0]= 12 11
/*
 * SPI registers bit definition ends here
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_Init(SPI_Handle_t *pSPIx);


/*
 * data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len);
/*
 *IRQ configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);					//priority config

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);		//stop transmission
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);			//stop receiving
/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F334XX_SPI_DRIVER_H_ */
