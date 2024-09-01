/*
 * stm32f446re_usart_driver.c
 *
 *  Created on: Apr 24, 2024
 *      Author: DM
 */

#include "stm32f446re_usart_driver.h"



static void USART_SetBaudRate( USART_RegDef_t *pUSARTx, uint32_t BaudRate){

	uint32_t PClk = 0, usartdiv = 0;
	uint32_t mantissa = 0, fraction = 0;
	if( (pUSARTx == USART1) || (pUSARTx == USART6))		PClk = GetAPB1Clock();
	else												PClk = GetAPB2Clock();

	if( pUSARTx->CR1.OVER8 == FLAG_RESET ){		// Over sampling by 16
		usartdiv = ( (25 * PClk) / ( 4 * BaudRate) );
	}else{										// Over sampling by 8
		usartdiv = ( (25 * PClk) / ( 2 * BaudRate) );
	}

	mantissa = usartdiv / 100;
	pUSARTx->BRR.DIV_MANTISSA = mantissa;

	fraction = ( usartdiv - (mantissa * 100) );
	if( pUSARTx->CR1.OVER8 == FLAG_RESET ){		// Over sampling by 16
		fraction = ( ( ( (fraction * 16) + 50 ) / 100 ) & (uint8_t) 0x0F );
	}else{										// Over sampling by 8
		fraction = ( ( ( (fraction * 8) + 50 ) / 100 ) & (uint8_t) 0x07 );
	}
	pUSARTx->BRR.DIV_FRACTION = fraction;

}

void USART_PeriClockControl(USART_RegDef_t * pUSARTx, uint8_t EnorDi){

	if( EnorDi == ENABLE){

		if( pUSARTx == USART1) 		USART1_CLOCK_EN();
		else if( pUSARTx == USART2) USART2_CLOCK_EN();
		else if( pUSARTx == USART3) USART3_CLOCK_EN();
		else if( pUSARTx == UART4) 	UART4_CLOCK_EN();
		else if( pUSARTx == UART5) 	UART5_CLOCK_EN();
		else if( pUSARTx == USART6) USART6_CLOCK_EN();

	}else{

		if( pUSARTx == USART1) 		USART1_CLOCK_DIS();
		else if( pUSARTx == USART2) USART2_CLOCK_DIS();
		else if( pUSARTx == USART3) USART3_CLOCK_DIS();
		else if( pUSARTx == UART4) 	UART4_CLOCK_DIS();
		else if( pUSARTx == UART5) 	UART5_CLOCK_DIS();
		else if( pUSARTx == USART6) USART6_CLOCK_DIS();

	}

}

void USART_Init(USART_Handle_t * USART_Handle){

	USART_PeriClockControl(USART_Handle->pUSARTx, ENABLE);

	// Set the USART/USART mode
	if( (USART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) || (USART_Handle->USART_Config.USART_Mode == USART_MODE_TX_RX) )
		USART_Handle->pUSARTx->CR1.TE = ENABLE;

	if( (USART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) || (USART_Handle->USART_Config.USART_Mode == USART_MODE_TX_RX) )
		USART_Handle->pUSARTx->CR1.RE = ENABLE;

	USART_Handle->pUSARTx->CR2.STOP = USART_Handle->USART_Config.USART_NumStopBits;
	USART_Handle->pUSARTx->CR1.M = USART_Handle->USART_Config.USART_TranferLength;

	if( USART_Handle->USART_Config.USART_ParityControl != USART_PARITY_NONE ){
		// Enable the parity control
		USART_Handle->pUSARTx->CR1.PCE = ENABLE;

		// Enable for EVEN or ODD parity in CR1 register: PS bit.
		if( USART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EVEN)		USART_Handle->pUSARTx->CR1.PS = DISABLE;
		else if( USART_Handle->USART_Config.USART_ParityControl == USART_PARITY_ODD )	USART_Handle->pUSARTx->CR1.PS = ENABLE;
	}else{
		USART_Handle->pUSARTx->CR1.PCE = DISABLE;
	}

	if( USART_Handle->USART_Config.USART_HWFlowControl != USART_HW_FLOWCTRL_NONE ){
		// Enable the Hardware Flow Control for RTS, CTS or both.
		if( (USART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_CTS) || (USART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_CTS_RTS))
			USART_Handle->pUSARTx->CR3.CTSE = ENABLE;

		if( (USART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_RTS) || (USART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOWCTRL_CTS_RTS))
				USART_Handle->pUSARTx->CR3.RTSE = ENABLE;
	}else{
		USART_Handle->pUSARTx->CR3.CTSE = DISABLE;
		USART_Handle->pUSARTx->CR3.RTSE = DISABLE;
	}

	// Configure the BRR register to set the Baud Rate
	USART_SetBaudRate(USART_Handle->pUSARTx, USART_Handle->USART_Config.USART_BaudRate);
}
void USART_DeInit(USART_RegDef_t * pUSARTx){

	if( pUSARTx == USART1 )				USART1_REG_RST();
	else if( pUSARTx == USART2 )		USART2_REG_RST();
	else if( pUSARTx == USART3 )		USART3_REG_RST();
	else if( pUSARTx == UART4 )			UART4_REG_RST();
	else if( pUSARTx == UART5 )			UART5_REG_RST();
	else if( pUSARTx == USART6 )		USART6_REG_RST();
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t BufferLen){

	while( BufferLen > 0){

		// Wait until the last data is transfered to the shift register
		while( pUSARTHandle->pUSARTx->SR.TXE == FLAG_RESET );

		if( pUSARTHandle->USART_Config.USART_TranferLength == USART_TRANSFERLENGTH_9BITS ){
			uint16_t * pData = NULL;
			pData = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = ( *pData & 0x01FF );

			if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE ){
				pTxBuffer += 2;
				BufferLen -= 2;
			}else{
				pTxBuffer ++;
				BufferLen --;
			}

		}else{

			pUSARTHandle->pUSARTx->DR = ( *pTxBuffer & 0xFF );
			pTxBuffer ++;
			BufferLen --;
		}

	}

	// Wait until TC (Transfer completed) flag is set.
	while( pUSARTHandle->pUSARTx->SR.TC == FLAG_RESET );

}

void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t BufferLen){

	while( BufferLen > 0 ){

		// Verify Data is ready to read
		while( pUSARTHandle->pUSARTx->SR.RXNE == FLAG_RESET );

		if( pUSARTHandle->USART_Config.USART_TranferLength == USART_TRANSFERLENGTH_9BITS ){

			if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE){
				*( (uint16_t*) pRxBuffer ) = (pUSARTHandle->pUSARTx->DR && (uint16_t)0x01FF);
				pRxBuffer += 2;
				BufferLen -= 2;
			}else{
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);	// Parity Enabled, so 8 bits is for data.
				pRxBuffer ++;
				BufferLen --;
			}

		}else{	// In this case we receive 8 bit for data

			if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE){
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);	// No parity Enabled, so 8 bits is for data.
			}else{
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);	// In this case 7 bits is for data and 1 bit for parity control.
			}
			pRxBuffer ++;
			BufferLen --;

		}

	}

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t BufferLen){
	uint8_t state = pUSARTHandle->TxBusyState;

	if( state != USART_BUSY_IN_TX){

		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = BufferLen;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupts
		// Enable TXE interrupt
		pUSARTHandle->pUSARTx->CR1.TXEIE = ENABLE; //  An USART interrupt is generated whenever TXE=1 in the USART_SR register
		//Enable TC interrupt
		pUSARTHandle->pUSARTx->CR1.TCIE = ENABLE; // An USART interrupt is generated whenever TC=1 in the USART_SR register

	}

	return state;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t BufferLen){

	uint8_t state = pUSARTHandle->RxBusyState;

	if( state != USART_BUSY_IN_RX){

		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = BufferLen;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Enable interrupt for RXE (Received data is ready to read)
		pUSARTHandle->pUSARTx->CR1.RXNEIE = ENABLE; 	//	An USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register

	}

	return state;
}

void USART_IRQHandling(USART_Handle_t * pUSARTHandle){

	// 1. Transmit Data Register Empty
	if( (pUSARTHandle->pUSARTx->SR.TXE) && (pUSARTHandle->pUSARTx->CR1.TXEIE) ){

		// Use to load a byte into DR register for the next transmission
		if( (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) && (pUSARTHandle->TxLen > 0) ){

			if( pUSARTHandle->USART_Config.USART_TranferLength == USART_TRANSFERLENGTH_9BITS ){
				uint16_t * pData = NULL;
				pData = (uint16_t*) pUSARTHandle->pTxBuffer;
				pUSARTHandle->pUSARTx->DR = ( *pData & 0x01FF );

				if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE ){
					pUSARTHandle->pTxBuffer += 2;
					pUSARTHandle->TxLen -= 2;
				}else{
					pUSARTHandle->pTxBuffer ++;
					pUSARTHandle->TxLen --;
				}

			}else{

				pUSARTHandle->pUSARTx->DR = ( *(pUSARTHandle->pTxBuffer) & 0xFF );
				pUSARTHandle->pTxBuffer ++;
				pUSARTHandle->TxLen --;
			}

		}

	}

	// 2. CTS Flag:  A change occurred on the nCTS status line
	if( (pUSARTHandle->pUSARTx->SR.CTS) && (pUSARTHandle->pUSARTx->CR3.CTSIE) ){
		// This bit is set by hardware when the nCTS input toggles, if the CTSE bit is set. It is cleared
		// by software (by writing it to 0). An interrupt is generated if CTSIE=1 in the USART_CR3
		// register.
		pUSARTHandle->pUSARTx->SR.CTS = FLAG_RESET;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENTCALLBACK_CTS_FLAG);

	}

	// 3. Transmission Complete
	if( (pUSARTHandle->pUSARTx->SR.TC) && (pUSARTHandle->pUSARTx->CR1.TCIE) ){
	// This bit is set by hardware if the transmission of a frame containing data is complete and if
	// TXE is set. An interrupt is generated if TCIE=1 in the USART_CR1 register. It is cleared by
	// a software sequence (a read from the USART_SR register followed by a write to the
	// USART_DR register). The TC bit can also be cleared by writing a '0' to it.
		if( (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) && (pUSARTHandle->TxLen == 0) && (pUSARTHandle->pUSARTx->SR.TXE) ){
			pUSARTHandle->pUSARTx->SR.TC = FLAG_RESET;

			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENTCALLBACK_TX_CMPLT);

			// Clear flag enable interrupts
			pUSARTHandle->pUSARTx->CR1.TXEIE = RESET;
			pUSARTHandle->pUSARTx->CR1.TCIE = RESET;

			// Restore Data frame for handle.
			pUSARTHandle->TxBusyState = USART_READY;
			pUSARTHandle->pTxBuffer = NULL;
			pUSARTHandle->TxLen = 0;

		}

	}

	// 4. Received data ready to be read
	if( (pUSARTHandle->pUSARTx->SR.RXNE) && (pUSARTHandle->pUSARTx->CR1.RXNEIE) ){
	// This bit is set by hardware when the content of the RDR shift register has been transferred
	// to the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1
	// register. It is cleared by a read to the USART_DR register. The RXNE flag can also be
	// cleared by writing a zero to it. This clearing sequence is recommended only for multibuffer
	// communication.
		if( (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) && (pUSARTHandle->RxLen > 0) ){

			if( pUSARTHandle->USART_Config.USART_TranferLength == USART_TRANSFERLENGTH_9BITS ){

				if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE){
					*( (uint16_t*) pUSARTHandle->pRxBuffer ) = (pUSARTHandle->pUSARTx->DR && (uint16_t)0x01FF);
					pUSARTHandle->pRxBuffer += 2;
					pUSARTHandle->RxLen -= 2;
				}else{
					*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);	// Parity Enabled, so 8 bits is for data.
					pUSARTHandle->pRxBuffer ++;
					pUSARTHandle->RxLen --;
				}

			}else{	// In this case we receive 8 bit for data

				if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_NONE){
					*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);	// No parity Enabled, so 8 bits is for data.
				}else{
					*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);	// In this case 7 bits is for data and 1 bit for parity control.
				}
				pUSARTHandle->pRxBuffer ++;
				pUSARTHandle->RxLen --;
			}

		}

		if( (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) && (pUSARTHandle->RxLen == 0) ){
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENTCALLBACK_RX_CMPLT);

			// Clear flag enable interrupts
			pUSARTHandle->pUSARTx->CR1.RXNEIE = RESET;

			// Restore Data frame for handle.
			pUSARTHandle->RxBusyState = USART_READY;
			pUSARTHandle->pRxBuffer = NULL;
			pUSARTHandle->RxLen = 0;
		}

	}

	// 5. Overrun detected
	if( (pUSARTHandle->pUSARTx->SR.ORE) && (pUSARTHandle->pUSARTx->CR1.RXNEIE) ){
	//This bit is set by hardware when the word currently being received in the shift register is
	//ready to be transferred into the RDR register while RXNE=1. An interrupt is generated if
	//RXNEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
	//USART_SR register followed by a read to the USART_DR register).
		uint16_t data = pUSARTHandle->pUSARTx->DR;
		(void) data;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENTCALLBACK_OVERRRUN);
	}

	// 6. IDLE line detected
	if( (pUSARTHandle->pUSARTx->SR.IDLE) && (pUSARTHandle->pUSARTx->CR1.IDLEIE) ){
	// This bit is set by hardware when an Idle Line is detected. An interrupt is generated if the
	//IDLEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
	//USART_SR register followed by a read to the USART_DR register).
		uint16_t data = pUSARTHandle->pUSARTx->DR;
		(void) data;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENTCALLBACK_IDLE);
	}

	// 7.  Parity Error
	if( (pUSARTHandle->pUSARTx->SR.PE) && (pUSARTHandle->pUSARTx->CR1.PEIE) ){

	}

	// 8. Break Flag
	if( (pUSARTHandle->pUSARTx->SR.LBD) && (pUSARTHandle->pUSARTx->CR2.LBDIE) ){

	}

	// 9. Noise flag

}


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if( EnorDi == ENABLE){

		if( IRQNumber <= 31 ){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if( (IRQNumber > 31) && (IRQNumber <= 63) ){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if( (IRQNumber > 63) && (IRQNumber <= 95) ){
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	}else{

		if( IRQNumber <= 31 ){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if( (IRQNumber > 31) && (IRQNumber <= 63) ){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if( (IRQNumber > 63) && (IRQNumber <= 95) ){
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}

	}

}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - 4) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

__attribute__ ((weak)) void USART_ApplicationEventCallback(USART_Handle_t * pUSARTHandle, uint8_t Event){

}

void USART_EnablePer(USART_RegDef_t * pUSARTx, uint8_t EnorDi){
	if( EnorDi == ENABLE )			pUSARTx->CR1.UE = ENABLE;
	else if( EnorDi == DISABLE)		pUSARTx->CR1.UE = DISABLE;
}
