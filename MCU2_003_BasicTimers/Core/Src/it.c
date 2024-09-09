/*
 * it.c
 *
 *  Created on: Apr 30, 2024
 *      Author: DM
 */

#include "main.h"

void SysTick_Handler(void){

	HAL_IncTick();

}


void USART2_IRQHandler(void){

	extern UART_HandleTypeDef husart2;
	HAL_UART_IRQHandler(&husart2);

}


void TIM6_DAC_IRQHandler(void){

	extern TIM_HandleTypeDef htim6;
	HAL_TIM_IRQHandler(&htim6);
}
