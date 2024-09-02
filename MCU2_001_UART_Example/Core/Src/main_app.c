/*
 * main_app.c
 *
 *  Created on: Apr 30, 2024
 *      Author: DM
 */

#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "main.h"

UART_HandleTypeDef husart2 = {0};

char * text = "MCU 2 UART Example\n";
uint8_t data [120] = {0}, rcv_data;

int main(void){

	HAL_Init();

	SystemClock_Config();
	UART2_Init();

	HAL_UART_Transmit(&husart2, (uint8_t*) "MCU 2 UART Example\n", strlen("MCU 2 UART Example\n"), HAL_MAX_DELAY);

	uint16_t count = 0;

	while( true ){

//		HAL_UART_Receive(&husart2, &rcv_data, 1, HAL_MAX_DELAY);
//
//		if( rcv_data == '\r'){
//
//			data[count++] = '#';
//			data[count] = '\0';
//			str_to_uppercase(data, strlen((char *)data));
//			HAL_UART_Transmit(&husart2, data, strlen((char *)data), HAL_MAX_DELAY);
//			count = 0;
//		}else{
//			data[count] = rcv_data;
//			count ++;
//		}


		HAL_UART_Receive_IT(&husart2, &rcv_data, 1);
	}

	return 0;
}


void SystemClock_Config(){

}

void UART2_Init(void){

	husart2.Instance = USART2;

	husart2.Init.BaudRate = 115200;
	husart2.Init.Mode = UART_MODE_TX_RX;
	husart2.Init.StopBits = UART_STOPBITS_1;
	husart2.Init.Parity = UART_PARITY_NONE;
	husart2.Init.WordLength = UART_WORDLENGTH_8B;
	husart2.Init.OverSampling = UART_OVERSAMPLING_16;
	husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	if( HAL_UART_Init(&husart2) == HAL_ERROR )		while(1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	static uint16_t count = 0;

	if( rcv_data == '\r'){

		data[count++] = '#';
		data[count] = '\0';
		str_to_uppercase(data, strlen((char *)data));
		HAL_UART_Transmit(huart, data, strlen((char *)data), HAL_MAX_DELAY);
		count = 0;
	}else{
		data[count] = rcv_data;
		count ++;
	}



}

void str_to_uppercase(uint8_t * str, uint16_t len){

	uint8_t offset = 'A' - 'a';
	for( uint16_t i = 0 ; i < len; i ++ ){

		if( ( *str >= 'a' ) && ( *str <= 'z' ) ){
			*str += offset;
		}else if( ( *str >= 'A' ) && ( *str <= 'Z' ) ){
			*str -= offset;
		}
		str ++;


	}

}
