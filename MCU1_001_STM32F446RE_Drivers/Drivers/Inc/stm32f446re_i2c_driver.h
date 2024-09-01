/*
 * stm32f446re_i2c_driver.h
 *
 *  Created on: Apr 21, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_I2C_DRIVER_H_
#define INC_STM32F446RE_I2C_DRIVER_H_

#include "stm32f446re.h"

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t * pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffer;
	uint16_t TxLen;
	uint16_t RxLen;
	uint8_t SDAState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t StartRep;
}I2C_Handle_t;


typedef enum{
	I2C_SCL_SPEED_STANDARD = 100000,
	I2C_SCL_SPEED_FAST	= 400000,
}I2C_SCLSpeed_e;

typedef enum{
	I2C_ACK_DISABLE = 0,
	I2C_ACK_ENABLE = 1,
}I2C_ACKControl_e;

typedef enum{
	I2C_FMDUTY_2,
	I2C_FMDUTY_16_9,
}I2C_FMDuty_e;

typedef enum{
	I2C_START_REPEAT_DISABLE,
	I2C_START_REPEAT_ENABLE,
}I2C_StartRep_e;

typedef enum{
	I2C_SDA_READY,
	I2C_SDA_BUSY_IN_TX,
	I2C_SDA_BUSY_IN_RX,
}I2C_SDAState_e;

typedef enum{
	I2C_EVENTCALLBACK_TX_CMPLT,
	I2C_EVENTCALLBACK_RX_CMPLT,
	I2C_EVENTCALLBACK_STOP,
	I2C_EVENTCALLBACK_DATA_REQUEST,
	I2C_EVENTCALLBACK_DATA_RECEIVE,
	I2C_EVENTCALLBACK_ERROR_BERR,
	I2C_EVENTCALLBACK_ERROR_ARLO,
	I2C_EVENTCALLBACK_ERROR_AF,
	I2C_EVENTCALLBACK_ERROR_OVR,
	I2C_EVENTCALLBACK_ERROR_PECERR,
	I2C_EVENTCALLBACK_ERROR_TIMEOUT,
	I2C_EVENTCALLBACK_ERROR_SMBALERT,
}I2C_IT_EventCallback;

typedef enum{
	I2C_MODE_SLAVE_FLAG,
	I2C_MODE_MASTER_FLAG,
}I2C_ModeFlag_e;

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t * pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data read and write
 */
void I2C_MasterSendData(I2C_RegDef_t *pI2Cx, uint8_t* pTxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep);
void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t* pRxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
/*
 * Data read and write for interrupts
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t * pI2CHanlde, uint8_t* pTxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t * pI2CHanlde, uint8_t* pRxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t * pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t * pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t * pI2CHandle, uint8_t Event);

void I2C_EnablePer(I2C_RegDef_t * pI2Cx, uint8_t EnorDi);
void I2C_EnableACK(I2C_RegDef_t * pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F446RE_I2C_DRIVER_H_ */
