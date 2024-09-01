/*
 * 015_USART_TX_IT_Testing.c
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446re.h"

void Button_Config(void);
void USART_pinsConfig(void);
void USARTConfig(void);

USART_Handle_t usart = {0};
char * my_text = "Hello from stm32f446re using USART protocol \n";
uint8_t rx_data[100] = {0};

int main(void){

	Button_Config();
	USARTConfig();
	USART_pinsConfig();

	USART_IRQInterruptConfig(IRQ_NUM_USART1, ENABLE);
	USART_IRQPriorityConfig(IRQ_NUM_USART1, 15);

	USART1->CR1.UE = ENABLE;


	while(1){

		if( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == RESET){

			for(uint32_t i = 0 ; i < 500000/2 ; i ++);

			USART_ReceiveDataIT(&usart, rx_data, strlen(my_text));
			USART_SendData(&usart, (uint8_t*) my_text, strlen(my_text));

		}

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
	usart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&usart_pins);

}

void USARTConfig(void){

	usart.pUSARTx = USART1;
	usart.USART_Config.USART_BaudRate = USART_BAUDRATE_115200;
	usart.USART_Config.USART_Mode = USART_MODE_TX_RX;
	usart.USART_Config.USART_HWFlowControl = USART_HW_FLOWCTRL_NONE;
	usart.USART_Config.USART_NumStopBits = USART_NUMSTOPBITS_1;
	usart.USART_Config.USART_TranferLength = USART_TRANSFERLENGTH_8BITS;
	usart.USART_Config.USART_ParityControl = USART_PARITY_NONE;

	USART_Init(&usart);

}

void Button_Config(void){

	GPIO_Handle_t user_button = {0};

	user_button.pGPIOx = GPIOC;
	user_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	user_button.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0;
	user_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	user_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	user_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&user_button);

}

void USART1_IRQHandler(void){

	USART_IRQHandling(&usart);

}

void USART_ApplicationEventCallback(USART_Handle_t * pUSARTHandle, uint8_t Event){

	if( Event == USART_EVENTCALLBACK_RX_CMPLT ){
		printf((char *)rx_data);
	}

}
