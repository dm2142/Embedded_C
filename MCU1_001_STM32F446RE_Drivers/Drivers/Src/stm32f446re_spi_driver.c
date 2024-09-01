/*
 * stm32f446re_spi_driver.c
 *
 *  Created on: Apr 15, 2024
 *      Author: DM
 */


#include "stm32f446re_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if( EnorDi == ENABLE ){

		if(pSPIx == SPI1)		SPI1_CLOCK_EN();
		else if(pSPIx == SPI2)	SPI2_CLOCK_EN();
		else if(pSPIx == SPI3)	SPI3_CLOCK_EN();
		else if(pSPIx == SPI4)	SPI4_CLOCK_EN();

	}else{

		if(pSPIx == SPI1)		SPI1_CLOCK_DIS();
		else if(pSPIx == SPI2)	SPI2_CLOCK_DIS();
		else if(pSPIx == SPI3)	SPI3_CLOCK_DIS();
		else if(pSPIx == SPI4)	SPI4_CLOCK_DIS();

	}

}

void SPI_Init(SPI_Handle_t * pSPIHanlde){

	// Enable the clock peripheral
	SPI_PeriClockControl(pSPIHanlde->pSPIx, ENABLE);

	if(pSPIHanlde->SPI_Congif.SPI_BusConfig == SPI_SIMPLEX_RXONLY){
		pSPIHanlde->pSPIx->CR1.BIDIMODE = SPI_FULLDUPLEX;
		pSPIHanlde->pSPIx->CR1.RXONLY = 1;
	}else{
		pSPIHanlde->pSPIx->CR1.BIDIMODE = pSPIHanlde->SPI_Congif.SPI_BusConfig;
	}

	pSPIHanlde->pSPIx->CR1.MSTR = pSPIHanlde->SPI_Congif.SPI_DeviceMode;
	pSPIHanlde->pSPIx->CR1.BR = pSPIHanlde->SPI_Congif.SPI_SCKLSpeed;
	pSPIHanlde->pSPIx->CR1.DFF = pSPIHanlde->SPI_Congif.SPI_DataFormat;
	pSPIHanlde->pSPIx->CR1.LSBFIRST = pSPIHanlde->SPI_Congif.SPI_FrameFormat;
	pSPIHanlde->pSPIx->CR1.SSM = pSPIHanlde->SPI_Congif.SPI_SSM;
	pSPIHanlde->pSPIx->CR1.CPHA = pSPIHanlde->SPI_Congif.SPI_CPHA;
	pSPIHanlde->pSPIx->CR1.CPOL = pSPIHanlde->SPI_Congif.SPI_CPOL;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1)		SPI1_REG_RST();
	else if(pSPIx == SPI2)	SPI2_REG_RST();
	else if(pSPIx == SPI3)	SPI3_REG_RST();
	else if(pSPIx == SPI4)	SPI4_REG_RST();

}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t BufferLen){

	while( BufferLen > 0){

		while( pSPIx->SR.TXE == RESET);

		if(pSPIx->CR1.DFF == SPI_16_DATAFORMAT){

			pSPIx->DR = *((uint16_t*) pTxBuffer);
			BufferLen -= 2;
			pTxBuffer += 2;

		}else{

//			*((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
			pSPIx->DR = *pTxBuffer;
			BufferLen --;
			pTxBuffer ++;

		}

	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t BufferLen){

	while( BufferLen > 0){

		while( pSPIx->SR.RXNE == RESET);

		if(pSPIx->CR1.DFF == SPI_16_DATAFORMAT){

			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			BufferLen -= 2;
			pRxBuffer += 2;

		}else{

//			*((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
			*pRxBuffer = pSPIx->DR;
			BufferLen --;
			pRxBuffer ++;

		}

	}

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if( EnorDi == ENABLE){

		if( IRQNumber <= 31 )								*NVIC_ISER0 |= (1 << IRQNumber);
		else if( (IRQNumber > 31) && (IRQNumber <= 63) )	*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		else if( (IRQNumber > 63) && (IRQNumber <= 95) )	*NVIC_ISER2 |= (1 << (IRQNumber % 64));

	}else{

		if( IRQNumber <= 31 )								*NVIC_ICER0 |= (1 << IRQNumber);
		else if( (IRQNumber > 31) && (IRQNumber <= 63) )	*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		else if( (IRQNumber > 63) && (IRQNumber <= 95) )	*NVIC_ICER2 |= (1 << (IRQNumber % 64));

	}

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - 4) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void SPI_EnablePer(SPI_RegDef_t * pSPIx, uint8_t EnorDi){
	if( EnorDi == ENABLE )			pSPIx->CR1.SPE = ENABLE;
	else if( EnorDi == DISABLE)		pSPIx->CR1.SPE = DISABLE;
}

void SPI_EnableSlaveSelectOutput(SPI_RegDef_t * pSPIx, uint8_t EnorDi){
	if( EnorDi == ENABLE )			pSPIx->CR2.SSOE = ENABLE;
	else if( EnorDi == DISABLE)		pSPIx->CR2.SSOE = DISABLE;
}
