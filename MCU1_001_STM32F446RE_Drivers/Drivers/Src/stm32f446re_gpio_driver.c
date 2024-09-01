/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: Apr 13, 2024
 *      Author: DM
 */

#include "stm32f446re_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if( EnorDi == ENABLE ){

		if(pGPIOx == GPIOA)			GPIOA_CLOCK_EN();
		else if (pGPIOx == GPIOB)	GPIOB_CLOCK_EN();
		else if (pGPIOx == GPIOC)	GPIOC_CLOCK_EN();
		else if (pGPIOx == GPIOD)	GPIOD_CLOCK_EN();
		else if (pGPIOx == GPIOE)	GPIOE_CLOCK_EN();
		else if (pGPIOx == GPIOF)	GPIOF_CLOCK_EN();
		else if (pGPIOx == GPIOG)	GPIOG_CLOCK_EN();
		else if (pGPIOx == GPIOH)	GPIOH_CLOCK_EN();


	}else
	{

		if(pGPIOx == GPIOA)			GPIOA_CLOCK_DIS();
		else if (pGPIOx == GPIOB)	GPIOB_CLOCK_DIS();
		else if (pGPIOx == GPIOC)	GPIOC_CLOCK_DIS();
		else if (pGPIOx == GPIOD)	GPIOD_CLOCK_DIS();
		else if (pGPIOx == GPIOE)	GPIOE_CLOCK_DIS();
		else if (pGPIOx == GPIOF)	GPIOF_CLOCK_DIS();
		else if (pGPIOx == GPIOG)	GPIOG_CLOCK_DIS();
		else if (pGPIOx == GPIOH)	GPIOH_CLOCK_DIS();
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t aux = 0; 		// Use auxiliary variable

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	if( (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode >= GPIO_MODE_INPUT) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  <= GPIO_MODE_ANALOG) ){

		aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );				// Clear bit positions.
		pGPIOHandle->pGPIOx->MODER |= aux;

	}else{
		// For INTERRUPT configuration

		aux = (GPIO_MODE_INPUT << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= aux; //setting

		// Configure the Rising/Falling edge trigger for interrupt
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			// Set interrupt for Rising trigger
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT ){

			// Set interrupt for Falling trigger
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ){

			// Set interrupt for both rising and falling trigger
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// Configure the GPIO PORT selection in SYSCFG register

		uint8_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t position = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t value = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_CLOCK_EN();
		SYSCFG->EXTICR[index] |= ( value << (4 * position));

		// Enable the interrupt mask register for.
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}


	// Configuration the output speed
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEED &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEED |= aux;

	// Configuration the pull up or pull down registers
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR |= aux;

	// Configure the output type
	aux = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER |= aux;

	// Configure the alternate function if is selected.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUN){

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );

	}


}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RST();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RST();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RST();
	}

}


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t state = 0;
	state = ( (pGPIOx->IDR) >> PinNumber) & 0x01;
	return state;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value = 0x0000;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - 4) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t PinNumber){

	// Clear the EXTI interrupt corresponding with the pin.
	if( EXTI->PR & (1 << PinNumber) ){
		// PRx bit is cleared by programming it to '1'
		EXTI->PR |= (1 << PinNumber);
	}

}
