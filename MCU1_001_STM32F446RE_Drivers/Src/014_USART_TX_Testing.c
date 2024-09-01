/*
 * 014_USART_TX_Testing.c
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446re.h"

void USART_pinsConfig(void);
void USARTConfig(void);

USART_Handle_t usart = {0};
char * my_text = "Hello from stm32f446re using USART protocol \n";

int main(void){

	USARTConfig();
	USART_pinsConfig();

	USART1->CR1.UE = ENABLE;


	while(1){

	for(uint32_t i = 0; i < 500000; i++);
	USART_SendData(&usart, (uint8_t *)my_text, strlen(my_text));

	}

}

void USART_pinsConfig(void){

	GPIO_Handle_t usart_pins = {0};
	usart_pins.pGPIOx = GPIOA;

	usart_pins.GPIO_PinConfig.GPIO_PinAltFunMode =GPIO_AF_7;
	usart_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	usart_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;
	usart_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;
	usart_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	// Tx pin
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&usart_pins);

	// Rx pin
//	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
//	GPIO_Init(&usart_pins);

}

void USARTConfig(void){

	usart.pUSARTx = USART1;
	usart.USART_Config.USART_BaudRate = USART_BAUDRATE_115200;
	usart.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart.USART_Config.USART_HWFlowControl = USART_HW_FLOWCTRL_NONE;
	usart.USART_Config.USART_NumStopBits = USART_NUMSTOPBITS_1;
	usart.USART_Config.USART_TranferLength = USART_TRANSFERLENGTH_8BITS;
	usart.USART_Config.USART_ParityControl = USART_PARITY_NONE;

	USART_Init(&usart);

}
