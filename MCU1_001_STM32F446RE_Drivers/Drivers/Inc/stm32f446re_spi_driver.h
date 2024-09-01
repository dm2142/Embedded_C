/*
 * stm32f446re_spi_driver.h
 *
 *  Created on: Apr 15, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include "stm32f446re.h"

/*	***************************************************************************
 *  ****************** HANDLE STRUCTURES FOR SPI ******************************
 *  ***************************************************************************
 */

#define	SPI_READY			0
#define	SPI_BUSY_IN_TX		1
#define SPI_BUSY_IN_RX		2

typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCKLSpeed;
	uint8_t SPI_DataFormat;
	uint8_t SPI_FrameFormat;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

} SPI_Config_t;

typedef struct{

	SPI_RegDef_t * pSPIx;
	SPI_Config_t SPI_Congif;
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;

/*
 * ********************************************************
 * 				FUNCTIONS DECLARATIONS FOR SPI
 * ********************************************************
 */

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t * pSPIHanlde);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data read and write
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t BufferLen);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t BufferLen);


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t * pSPIHandle);

void SPI_EnablePer(SPI_RegDef_t * pSPIx, uint8_t EnorDi);
void SPI_EnableSlaveSelectOutput(SPI_RegDef_t * pSPIx, uint8_t EnorDi);
/*
 * ********************************************************
 * 				ENUMS FOR SPI CONFIGURATION
 * ********************************************************
 */

typedef enum{
	SPI_SLAVE,
	SPI_MASTER
}SPI_DeviceMode_e;

typedef enum{
	SPI_FULLDUPLEX,
	SPI_HALFDUPLEX,
	SPI_SIMPLEX_RXONLY
}SPI_BusConfig_e;

typedef enum{
	SPI_CLOCK_DIV_2,
	SPI_CLOCK_DIV_4,
	SPI_CLOCK_DIV_8,
	SPI_CLOCK_DIV_16,
	SPI_CLOCK_DIV_32,
	SPI_CLOCK_DIV_64,
	SPI_CLOCK_DIV_128,
	SPI_CLOCK_DIV_256,
}SPI_ClockSpeed_e;

typedef enum{
	SPI_8_DATAFORMAT,
	SPI_16_DATAFORMAT,
}SPI_DataFormat_e;

typedef enum{
	SPI_MSB_TRANSMIT_FIRST,
	SPI_LSB_TRANSMIT_FIRST,
}SPI_FrameFormat_e;

typedef enum{
	SPI_CPOL_LOW,
	SPI_CPOL_HIGH,
}SPI_ClokcPolarity_e;

typedef enum{
	SPI_CPHA_FIRST_EDGE,
	SPI_CPHA_SECOND_EDGE
}SPI_ClokcPhase_e;

typedef enum{
	SPI_SSM_DISABLED,
	SPI_SSM_ENABLED,
}SPI_SW_Slave_Managment_e;

#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
