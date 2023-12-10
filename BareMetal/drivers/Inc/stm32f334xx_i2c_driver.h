/*
 * stm32f334xx_i2c_driver.h
 *
 *  Created on: 24-Apr-2023
 *      Author: adeps
 */

#ifndef INC_STM32F334XX_I2C_DRIVER_H_
#define INC_STM32F334XX_I2C_DRIVER_H_

#include "stm32f334xx.h"

//I2C related flag status definition
#define I2C_FLAG_TXE 	(1 << I2C_ISR_TXE)
#define I2_FLAG_RXNE 	(1 << I2C_ISR_RXNE )
#define I2C_FLAG_SB		(1 << I2C_ISR_SB)


//I2C configuration structure
typedef struct
{
	uint32_t I2C_SCLSpeed;		//to configure the clock speed
	uint8_t I2C_DeviceAddress;	//to configure the device address
	uint8_t I2C_AckControl;	//to configure the ACK control
	uint8_t I2C_FMDutyCycle;	//to configure the duty cycle
}I2C_Config_t;


//I2C handle structure
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM1K	100000	//standard	mode 100kHz
#define I2C_SCL_SPEED_FM4K	400000	//fast 		mode 400kHz
#define I2C_SCL_SPEED_FM1M	1000000	//fas-pluse	mode 1MHz

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE
#define I2C_ACK_DISABLE

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2
#define I2C_FM_DUTY_16

/*
 * I2C peripheral clock
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * I2C init and deinit
 */
void I2C_init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

/*
 * I2C send receive
 */
void I2C_SendData(I2C_RegDef_t *pI2Cx,uint8_t *pTxBuffer,uint32_t len);
void I2C_ReceiveData(I2C_RegDef_t *pI2Cx,uint8_t *pTxBuffer,uint32_t len);

void I2C_SendDataIT(I2C_RegDef_t *pI2Cx,uint8_t *pTxBuffer,uint32_t len);
void I2C_ReceiveDataIT(I2C_RegDef_t *pI2Cx,uint8_t *pTxBuffer,uint32_t len);

/*
 * Interrupt related
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
uint8_t I2CGetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t appEv);

#endif /* INC_STM32F334XX_I2C_DRIVER_H_ */
