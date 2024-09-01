/*
 * stm32f446re_usart_driver.h
 *
 *  Created on: Apr 24, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_USART_DRIVER_H_
#define INC_STM32F446RE_USART_DRIVER_H_

#include "stm32f446re.h"

typedef struct{
	uint8_t	 	USART_Mode;
	uint32_t 	USART_BaudRate;
	uint8_t		USART_NumStopBits;
	uint8_t 	USART_TranferLength;
	uint8_t		USART_ParityControl;
	uint8_t		USART_HWFlowControl;
}USART_Config_t;

typedef struct{
	USART_RegDef_t * pUSARTx;
	USART_Config_t USART_Config;
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffer;
	uint32_t  TxLen;
	uint32_t  RxLen;
	uint8_t   TxBusyState;
	uint8_t   RxBusyState;
}USART_Handle_t;

typedef enum{
	USART_MODE_TX_RX,
	USART_MODE_ONLY_RX,
	USART_MODE_ONLY_TX
}USART_Mode_e;

typedef enum{
	USART_BAUDRATE_1200 	= 1200,
	USART_BAUDRATE_2400		= 2400,
	USART_BAUDRATE_9600		= 9600,
	USART_BAUDRATE_19200	= 19200,
	USART_BAUDRATE_38400	= 38400,
	USART_BAUDRATE_57600	= 57600,
	USART_BAUDRATE_115200	= 115200,
	USART_BAUDRATE_230400	= 230400,
	USART_BAUDRATE_460800	= 460800,
	USART_BAUDRATE_921600	= 921600,
	USART_BAUDRATE_2M		= 2000000,
	USART_BAUDRATE_3M		= 3000000,
}USART_Baudrate_e;

typedef enum{
	USART_NUMSTOPBITS_1 	= 0,
	USART_NUMSTOPBITS_0_5 	= 1,
	USART_NUMSTOPBITS_2		= 2,
	USART_NUMSTOPBITS_1_5	= 3,
}USART_NumStopBits_e;

typedef enum{
	USART_PARITY_NONE,
	USART_PARITY_EVEN,
	USART_PARITY_ODD,
}USART_Parity_e;

typedef enum{
	USART_TRANSFERLENGTH_8BITS,
	USART_TRANSFERLENGTH_9BITS,
}USART_TransferLength_e;

typedef enum{
	USART_HW_FLOWCTRL_NONE,
	USART_HW_FLOWCTRL_CTS,
	USART_HW_FLOWCTRL_RTS,
	USART_HW_FLOWCTRL_CTS_RTS,
}USART_HWFlowControl_e;

typedef enum{
	USART_BUSY_IN_TX,
	USART_BUSY_IN_RX,
	USART_READY,
}USART_State_e;

typedef enum{
	USART_EVENTCALLBACK_TX_CMPLT,
	USART_EVENTCALLBACK_RX_CMPLT,
	USART_EVENTCALLBACK_CTS_FLAG,
	USART_EVENTCALLBACK_OVERRRUN,
	USART_EVENTCALLBACK_IDLE,
	USART_EVENTCALLBACK_PARITY_ERR,
}USART_EventCallback_e;

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t * USART_Handle);
void USART_DeInit(USART_RegDef_t * pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t BufferLen);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t BufferLen);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t BufferLen);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t BufferLen);

void USART_IRQHandling(USART_Handle_t * pUSARTHandle);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void USART_ApplicationEventCallback(USART_Handle_t * pUSARTHandle, uint8_t Event);

void USART_EnablePer(USART_RegDef_t * pUSARTx, uint8_t EnorDi);

#endif /* INC_STM32F446RE_USART_DRIVER_H_ */
