/*
 * STM32F334xx_usart_driver.h
 *
 *  Created on: 27-Apr-2023
 *      Author: adep
 */

#ifndef STM32F334X_UART_DRIVER_H_
#define STM32F334X_UART_DRIVER_H_

#include "stm32f334xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t   USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;



/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 *USE more than one stop bits if baud rate is high > 9600 bps
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_ISR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_ISR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_ISR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0


#define 	USART_EVENT_TX_CMPLT   	0
#define		USART_EVENT_RX_CMPLT   	1
#define		USART_EVENT_IDLE       	2
#define		USART_EVENT_CTS        	3
#define		USART_EVENT_PE         	4
#define		USART_ERR_FE    		5
#define		USART_ERR_NE			6
#define		USART_ERR_ORE    		7

/*
 * USART ISR bit definitions
 */
#define USART_ISR_PE	0
#define USART_ISR_FE	1
#define USART_ISR_NF	2
#define USART_ISR_ORE	3
#define USART_ISR_IDLE	4
#define USART_ISR_RXNE	5
#define USART_ISR_TC	6
#define USART_ISR_TXE	7
#define USART_ISR_LBDF	8
#define USART_ISR_CTSIF	9
#define USART_ISR_CTS	10
#define USART_ISR_RTOF	11
#define USART_ISR_EOBF	12
#define USART_ISR_ARBE	14
#define USART_ISR_ABRF	15
#define USART_ISR_BUSY	16
#define USART_ISR_CMF	17
#define USART_ISR_RWU	18
#define USART_ISR_WUF	19
#define USART_ISR_TEACK	21
#define USART_ISR_REACK	22

/*
 * USART CR1 bit definitions
 */
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
 * USART CR2 bit definitions
 */
#define USART_CR2_ADDM	4
#define USART_CR2_LBDL	5
#define USART_CR2_LBDIE	6
#define USART_CR2_LBCL	8
#define USART_CR2_CPHA	9
#define USART_CR2__CPOL	10
#define USART_CR2_CLKEN	11
#define USART_CR2_STOP	12
#define USART_CR2_LINEN	14
#define USART_CR2_SWAP	15
#define USART_CR2_RXINV	16
#define USART_CR2_TXINV	17
#define USART_CR2_DATAINV	18
#define USART_CR2_MSBFIRST	19
#define USART_CR2_ABREN		20
#define USART_CR2_ABRMOD	21
#define USART_CR2_RTOEN		23
#define USART_CR2_ADD		24

/*
 * USART CR3 bit definition
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
#define USART_CR3_OVERDIS	12
#define USART_CR3_DDRE		13
#define USART_CR3_DEM		14
#define USART_CR3_DEP		15
#define USART_CR3_SCARCNT0	17
#define USART_CR3_SCARCNT1	18
#define USART_CR3_SCARCNT2	19
#define USART_CR3_WUS0		20
#define USART_CR3_WUS1		21
#define USART_CR3_WUFIE		22

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Init and DeInit USART
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * USART send and receive without interrupt
 */
void USART_SendData(USART_Handle_t *pUSART, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * USART send and receive using interrupt
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);


/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);


#endif /* STM32F446X_UART_DRIVER_H_ */


