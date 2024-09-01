/*
 * stm32f446re_i2c_driver.c
 *
 *  Created on: Apr 21, 2024
 *      Author: DM
 */

#include "stm32f446re_i2c_driver.h"

//static uint32_t GetAPB1Clock(void){
//
//	uint32_t clock_freq, sys_clk;
//	uint8_t aux, APB1_pres;
//	uint16_t AHB_pres;
//
//	aux = ( (RCC->CFGR >> 2) & 0b11 );
//
//	if( aux == 0b00 )		sys_clk = 16000000;	// HSI Oscillator set;
//	else if( aux == 0b01 )	sys_clk =  8000000;	// HSE Oscillator set;
//	else if( aux == 0b01 )	sys_clk =  0;	// PLL Oscillator set;
//	else if( aux == 0b01 )	sys_clk =  0;	// PLL R Oscillator set;
//
//	aux = ( (RCC->CFGR >> 4) & 0b1111 );
//	switch (aux) {
//		case 0b1000:	// System clock divided by 2
//			AHB_pres = 2;
//			break;
//		case 0b1001:	// System clock divided by 4
//			AHB_pres = 4;
//			break;
//		case 0b1010:	// System clock divided by 8
//			AHB_pres = 8;
//			break;
//		case 0b1011:	// System clock divided by 16
//			AHB_pres = 16;
//			break;
//		case 0b1100:	// System clock divided by 64
//			AHB_pres = 64;
//			break;
//		case 0b1101:	// System clock divided by 128
//			AHB_pres = 128;
//			break;
//		case 0b1110:	// System clock divided by 256
//			AHB_pres = 256;
//			break;
//		case 0b111:		// System clock divided by 512
//			AHB_pres = 512;
//			break;
//		default:		// System clock divided by 1
//			AHB_pres = 1;
//			break;
//	}
//
//	aux = ( (RCC->CFGR >> 10) & 0b111 );
//		switch (aux) {
//			case 0b100:	// System clock divided by 2
//				APB1_pres = 2;
//				break;
//			case 0b101:	// System clock divided by 4
//				APB1_pres = 4;
//				break;
//			case 0b110:	// System clock divided by 8
//				APB1_pres = 8;
//				break;
//			case 0b111:	// System clock divided by 16
//				APB1_pres = 16;
//				break;
//			default:		// System clock divided by 1
//				APB1_pres = 1;
//				break;
//		}
//
//	clock_freq = ( (sys_clk / AHB_pres) / APB1_pres );
//
//	return clock_freq;
//}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pI2Cx == I2C1) I2C1_CLOCK_EN();
		else if(pI2Cx == I2C2) I2C2_CLOCK_EN();
		else if(pI2Cx == I2C3) I2C3_CLOCK_EN();

	}else{

		if(pI2Cx == I2C1) I2C1_CLOCK_DIS();
		else if(pI2Cx == I2C2) I2C2_CLOCK_DIS();
		else if(pI2Cx == I2C3) I2C3_CLOCK_DIS();

	}

}

void I2C_Init(I2C_Handle_t * pI2CHandle){

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint16_t ccr_val = 0;
	uint32_t trise_val = 0;

	pI2CHandle->pI2Cx->CR1.ACK = pI2CHandle->I2C_Config.I2C_ACKControl;
	pI2CHandle->pI2Cx->CR2.FREQ = ( GetAPB1Clock()/1000000UL );
	pI2CHandle->pI2Cx->OAR1.ADD1 = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	pI2CHandle->pI2Cx->OAR1.HIGH_STATE = 1;		// Should always be kept bit 14 at 1 by software

	if( pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_STANDARD){

		pI2CHandle->pI2Cx->CCR.FS = 0;
		ccr_val = ( (GetAPB1Clock() ) / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		trise_val = ( GetAPB1Clock() / 1000000UL ) + 1;

	}else{	// I2C for Fast Mode Speed (400 KHz)

		pI2CHandle->pI2Cx->CCR.FS = 1;
		pI2CHandle->pI2Cx->CCR.DUTY = pI2CHandle->I2C_Config.I2C_FMDutyCycle;
		trise_val = ( (GetAPB1Clock() * 300) / 1000000000UL ) + 1;

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDUTY_2){

			ccr_val = ( (GetAPB1Clock() ) / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );

		}else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDUTY_16_9){

			ccr_val = ( (GetAPB1Clock() ) / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );

		}

	}

	pI2CHandle->pI2Cx->CCR.CCR = ccr_val;
	pI2CHandle->pI2Cx->TRISE = trise_val;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1) I2C1_REG_RST();
	else if(pI2Cx == I2C2) I2C2_REG_RST();
	else if(pI2Cx == I2C3) I2C3_REG_RST();

}

void I2C_MasterSendData(I2C_RegDef_t *pI2Cx, uint8_t* pTxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep){

	// 1-. Generate the START condition
	pI2Cx->CR1.START = ENABLE;

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( pI2Cx->SR1.SB == FLAG_RESET);

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	pI2Cx->DR = ( (SlaveAddr << 1 ) & ~(1) );

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( pI2Cx->SR1.ADDR == FLAG_RESET);

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	uint32_t aux;
	aux = pI2Cx->SR1.RESERVED1;
	aux = pI2Cx->SR2.RESERVED;
	(void) aux;

	//6. send the data until len. becomes 0
	while( BufferLen > 0){

		while( pI2Cx->SR1.TXE == FLAG_RESET);
		pI2Cx->DR = *pTxBuffer;
		pTxBuffer ++;
		BufferLen --;

	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while( (pI2Cx->SR1.TXE == FLAG_RESET) || (pI2Cx->SR1.BTF == FLAG_RESET) );

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if( StartRep == I2C_START_REPEAT_DISABLE) pI2Cx->CR1.STOP = ENABLE;

}

void I2C_MasterReceiveData(I2C_RegDef_t *pI2Cx, uint8_t* pRxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep){

	uint32_t aux;

	// 1. Generate the start condition.
	pI2Cx->CR1.START = ENABLE;

	// 2. Confirm that the start condition was completed by checking th SB flag in SR1 register.
	while( pI2Cx->SR1.SB == FLAG_RESET );

	// 3. Send the Address of the SLAVE with the R/W bit set to READ (1).
	pI2Cx->DR = ( (SlaveAddr << 1) | 0x1 );

	// 4. Wait until Address phase is completed by checking the ADDR flag in SR1 register.
	while( pI2Cx->SR1.ADDR == FLAG_RESET);

	// Steps to read one byte from slave.
	if( BufferLen == 1 ){

		// Disable ACK bit for CR1.
		pI2Cx->CR1.ACK = DISABLE;

		// Clear the ADDR flag by reading SR1 and SR2 registers.
		aux = pI2Cx->SR1.RESERVED1;
		aux = pI2Cx->SR2.RESERVED;
		(void) aux;

		// Wait until RXE flag is 1 (receive buffer is not empty).
		while( pI2Cx->SR1.RXNE == FLAG_RESET );

		// Generate the STOP condition.
		if( StartRep == I2C_START_REPEAT_DISABLE)	pI2Cx->CR1.STOP = ENABLE;

		// Read the data loaded from buffer into DR register.
		*pRxBuffer = pI2Cx->DR;

	}else{

		// Clear the ADDR flag by reading SR1 and SR2 registers.
		aux = pI2Cx->SR1.RESERVED1;
		aux = pI2Cx->SR2.RESERVED;
		(void) aux;

		while( BufferLen > 0 ){		// Read DATA according with the LEN.

			// Wait until RXE Flag becomes 1.
			while( pI2Cx->SR1.RXNE == FLAG_RESET );

			if( BufferLen == 2){	// When 2 bytes remaining

				// Clear the ACK flag.
				pI2Cx->CR1.ACK = DISABLE;

				// Generate the STOP condition.
				if( StartRep == I2C_START_REPEAT_DISABLE)	pI2Cx->CR1.STOP = ENABLE;
			}

			// Read the data from the data register into pRxBuffer
			*pRxBuffer = pI2Cx->DR;

			// Increment the pRxBuffer Address
			pRxBuffer ++;
			// Decrement the BufferLen
			BufferLen --;
		}

	}

	// Re-Enable ACK
	pI2Cx->CR1.ACK = ENABLE;

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return (uint8_t) pI2Cx->DR;
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t * pI2CHandle, uint8_t* pTxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep){
	uint8_t state = pI2CHandle->SDAState;

	if( state == I2C_SDA_READY ){

		// Enable the Interrupt flags

		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->TxLen = BufferLen;
		pI2CHandle->StartRep = StartRep;
		pI2CHandle->SDAState = I2C_SDA_BUSY_IN_TX;

		// Generate the START condition
		pI2CHandle->pI2Cx->CR1.START = ENABLE;

		// Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2.ITBUFEN = ENABLE;

		// Enable the ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2.ITEVTEN = ENABLE;

		// Enable the ITERREN Control bit
		pI2CHandle->pI2Cx->CR2.ITERREN = ENABLE;

	}

	return state;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t * pI2CHandle, uint8_t* pRxBuffer, uint32_t BufferLen, uint8_t SlaveAddr, uint8_t StartRep){

	uint8_t state = pI2CHandle->SDAState;

	if( state == I2C_SDA_READY ){

		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RxLen = BufferLen;
		pI2CHandle->RxSize = BufferLen;
		pI2CHandle->StartRep = StartRep;
		pI2CHandle->SDAState = I2C_SDA_BUSY_IN_RX;

		// Generate the START condition
		pI2CHandle->pI2Cx->CR1.START = ENABLE;

		// Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2.ITBUFEN = ENABLE;

		// Enable the ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2.ITEVTEN = ENABLE;

		// Enable the ITERREN Control bit
		pI2CHandle->pI2Cx->CR2.ITERREN = ENABLE;

	}

	return state;
}


void I2C_EV_IRQHandling(I2C_Handle_t * pI2CHandle){

	// Decode for the Event that rises the IRQ.
	// ITEVTEN Flag is set

	// 	1.	Start bit sent (Master mode) SB
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.SB == ENABLE) ){
		// Set when a start condition is generated (Only executed in master mode)

		// Lets execute the ADDRESS phase writing the SLAVE address.
		if( pI2CHandle->SDAState == I2C_SDA_BUSY_IN_TX){
			pI2CHandle->pI2Cx->DR = ( (pI2CHandle->DevAddr << 1 ) & ~(0x1) );	// Write
		}else if( pI2CHandle->SDAState == I2C_SDA_BUSY_IN_RX){
			pI2CHandle->pI2Cx->DR = ( (pI2CHandle->DevAddr << 1 ) | 0x1 );		// Read
		}

	}

	//	2.	Address sent (Master) or Address matched (Slave) ADDR
	// 	For MASTER mode indicates end of address transmission.
	// 	For SLAVE mode indicates end of address matched.
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.ADDR == ENABLE) ){

		// In the case if only receive one byte
		if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_MASTER_FLAG){

			if( (pI2CHandle->SDAState == I2C_SDA_BUSY_IN_RX) && (pI2CHandle->RxSize == 1) )		pI2CHandle->pI2Cx->CR1.ACK = DISABLE;

		}else if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_SLAVE_FLAG){	// For SLAVE mode

		}

		// Confirm that the phase ADDRESS was completed, clear the ADDR flag
		uint32_t aux;
		aux = pI2CHandle->pI2Cx->SR1.RESERVED1;
		aux = pI2CHandle->pI2Cx->SR2.RESERVED;
		(void) aux;
	}

	//	4.	Stop received (Slave) STOPF (Applies only for slave mode)
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.STOPBF == ENABLE) ){

		pI2CHandle->pI2Cx->CR1.ACK |= 0x0;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_STOP);

	}

	//	5.	Data byte transfer finished BTF (Byte transfer finished)
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.BTF == ENABLE) ){

		// Applies only for Master transmitter
		if( (pI2CHandle->SDAState == I2C_SDA_BUSY_IN_TX) && (pI2CHandle->pI2Cx->SR1.TXE == ENABLE) && (pI2CHandle->TxLen == 0) ){

			// Generate the Stop Condition if Start Repeated is not enabled
			if( pI2CHandle->StartRep == I2C_START_REPEAT_DISABLE )	pI2CHandle->pI2Cx->CR1.STOP = ENABLE;

			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_TX_CMPLT);

			// Reset the Member Elements in the Handle structure
			pI2CHandle->pI2Cx->CR2.ITBUFEN = DISABLE;
			pI2CHandle->pI2Cx->CR2.ITEVTEN = DISABLE;

			pI2CHandle->SDAState = I2C_SDA_READY;
			pI2CHandle->DevAddr = 0;
			pI2CHandle->pTxBuffer = NULL;
			pI2CHandle->TxLen = 0;

		}

	}

	//	ITEVTEN and ITBUFEN flags are set
	//	6.	Receive buffer not empty RxNE
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITBUFEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.RXNE == ENABLE) ){

		if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_MASTER_FLAG){

			if( (pI2CHandle->SDAState == I2C_SDA_BUSY_IN_RX) && (pI2CHandle->RxSize == 1) ){
				// Read the data loaded from buffer into DR register.
				*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxLen --;

			}else if( (pI2CHandle->SDAState == I2C_SDA_BUSY_IN_RX) && (pI2CHandle->RxSize > 1) ){

				if( pI2CHandle->RxLen == 2 ) pI2CHandle->pI2Cx->CR1.ACK = DISABLE;

				*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer ++;
				pI2CHandle->RxLen --;

			}

			if( pI2CHandle->RxLen == 0){
				// Generate the STOP condition.
				if( pI2CHandle->StartRep == I2C_START_REPEAT_DISABLE)	pI2CHandle->pI2Cx->CR1.STOP = ENABLE;

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_RX_CMPLT);

				// Reset Structure Members for I2CReceiveData
				pI2CHandle->pI2Cx->CR2.ITBUFEN = DISABLE;
				pI2CHandle->pI2Cx->CR2.ITEVTEN = DISABLE;

				pI2CHandle->SDAState = I2C_SDA_READY;
				pI2CHandle->DevAddr = 0;
				pI2CHandle->pRxBuffer = NULL;
				pI2CHandle->TxLen = 0;
				pI2CHandle->RxSize = 0;

				if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)		pI2CHandle->pI2Cx->CR1.ACK = ENABLE;

			}

		}else if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_SLAVE_FLAG ){

			// Check for device is in transmitter mode
			if( pI2CHandle->pI2Cx->SR2.TRA == RESET) 	 I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_DATA_RECEIVE);

		}

	}

	//	7. 	Transmit buffer empty TxE
	if( (pI2CHandle->pI2Cx->CR2.ITEVTEN == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITBUFEN == ENABLE) && (pI2CHandle->pI2Cx->SR1.TXE == ENABLE) ){

		// If TXE flag is set, data register is empty, so, load data for transmission

		if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_MASTER_FLAG ){

			if( (pI2CHandle->SDAState == I2C_SDA_BUSY_IN_TX) && (pI2CHandle->TxLen > 0) ){

				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				pI2CHandle->pTxBuffer ++;
				pI2CHandle->TxLen --;

			}

		}else if( pI2CHandle->pI2Cx->SR2.MSL == I2C_MODE_SLAVE_FLAG ){

			// Check for device is in transmitter mode
			if( pI2CHandle->pI2Cx->SR2.TRA == ENABLE) 	 I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_DATA_REQUEST);

		}

	}

}

void I2C_ER_IRQHandling(I2C_Handle_t * pI2CHandle){

	//	ITERREN Flag is set
	//	1. Bus error BERR	(Bus Error) Set by hardware when the interface detects an SDA rising or falling edge while SCL is high,
	//  occurring in a non-valid position during a byte transfer

	if( (pI2CHandle->pI2Cx->SR1.BERR == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){
		pI2CHandle->pI2Cx->SR1.BERR = 0;	//  Cleared by software writing 0, or by hardware when PE=0
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_BERR);
	}

	//	2. Arbitration loss (Master) ARLO Set by hardware when the interface loses the arbitration of the bus to another master
	if( (pI2CHandle->pI2Cx->SR1.ARLO == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){
		pI2CHandle->pI2Cx->SR1.ARLO = 0;	//  Cleared by software writing 0, or by hardware when PE=0
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_ARLO);
	}

	//	3. Acknowledge failure AF
	if( (pI2CHandle->pI2Cx->SR1.AF == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){
		pI2CHandle->pI2Cx->SR1.AF = 0;	//	Set by hardware when no acknowledge is returned.
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_AF);
	}

	//	4. Overrun/Underrun OVR
	// – In reception when a new byte is received (including ACK pulse) and the DR register has not
	//	been read yet. New received byte is lost.
	//	– In transmission when a new byte should be sent and the DR register has not been written
	//	yet. The same byte is sent twice
	if( (pI2CHandle->pI2Cx->SR1.OVR == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){
		pI2CHandle->pI2Cx->SR1.OVR = 0;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_OVR);
	}

	//	5. PEC error PECERR
	if( (pI2CHandle->pI2Cx->SR1.PECERR == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_PECERR);
	}

	//	6. Timeout/Tlow error TIMEOUT
	// 	– When set in slave mode: slave resets the communication and lines are released by hardware
	//	– When set in master mode: Stop condition sent by hardware
	if( (pI2CHandle->pI2Cx->SR1.TIMEOUT == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){
		pI2CHandle->pI2Cx->SR1.TIMEOUT = 0;
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_TIMEOUT);
	}

	//	7. SMBus Alert SMBALERT
	if( (pI2CHandle->pI2Cx->SR1.SMBALERT == ENABLE) && (pI2CHandle->pI2Cx->CR2.ITERREN == ENABLE)){

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENTCALLBACK_ERROR_SMBALERT);
	}

}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - 4) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

__attribute__ ((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t * pI2CHandle, uint8_t Event){

}

void I2C_EnablePer(I2C_RegDef_t * pI2Cx, uint8_t EnorDi){

	if( EnorDi == ENABLE )			pI2Cx->CR1.PE = ENABLE;
	else if( EnorDi == DISABLE)		pI2Cx->CR1.PE = DISABLE;
}

void I2C_EnableACK(I2C_RegDef_t * pI2Cx, uint8_t EnorDi){
	if( EnorDi == ENABLE )			pI2Cx->CR1.ACK = ENABLE;
	else if( EnorDi == DISABLE)		pI2Cx->CR1.ACK = DISABLE;
}
