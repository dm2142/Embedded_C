/*
 * main_app.c
 *
 *  Created on: Apr 30, 2024
 *      Author: DM
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "main.h"

static void RCC_Print_Clocks(void);
UART_HandleTypeDef husart2 = {0};

int main(void){

	HAL_Init();
//	SystemClock_Config((uint32_t)180e3);
//	UART2_Init();

	while( true ){

		SystemClock_Config((uint32_t)50e3);
		HAL_Delay(3000);

		SystemClock_Config((uint32_t)84e3);
		HAL_Delay(3000);

		SystemClock_Config((uint32_t)120e3);
		HAL_Delay(3000);

		SystemClock_Config((uint32_t)180e3);
		HAL_Delay(3000);

//		HAL_UART_Transmit(&husart2, (uint8_t*) "Running STM32f446RE @ 180 MHz \n\r", strlen("Running STM32f446RE @ 180 MHz \n\r"), HAL_MAX_DELAY);
//		HAL_Delay(1000);
	}

	return 0;
}


void SystemClock_Config(uint32_t ClockFreq){

	//PWR_REGULATOR_VOLTAGE_SCALE1         PWR_CR_VOS             /* Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to                                                                       180 MHz by activating the over-drive mode. */
	//PWR_REGULATOR_VOLTAGE_SCALE2         PWR_CR_VOS_1           /* Scale 2 mode: the maximum value of fHCLK is 144 MHz. It can be extended to                                                                       168 MHz by activating the over-drive mode. */
	//PWR_REGULATOR_VOLTAGE_SCALE3         PWR_CR_VOS_0           /* Scale 3 mode: the maximum value of fHCLK is 120 MHz. */

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	RCC_OscInitTypeDef osc_status = {0}, osc_init = {0};
	RCC_ClkInitTypeDef clk_init = {0};

	HAL_RCC_GetOscConfig(&osc_status);

	// Turn off PLL before change its configuration
	if( osc_status.PLL.PLLState == RCC_PLL_ON){


		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

		// Change to HSI clock as SYSCLOCK
		if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK)		Error_Handler();

		osc_init.OscillatorType = RCC_OSCILLATORTYPE_NONE;
		osc_init.PLL.PLLState = RCC_PLL_OFF;

		// Turn OFF PLL
		if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK)		Error_Handler();

		if (HAL_PWREx_DisableOverDrive() != HAL_OK)							Error_Handler();
		__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	}

	switch (ClockFreq) {
		case (uint32_t)180e3:

				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
				if (HAL_PWREx_EnableOverDrive() != HAL_OK)							Error_Handler();

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
				osc_init.HSEState = RCC_HSE_ON;
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
				osc_init.PLL.PLLM = 4;
				osc_init.PLL.PLLN = 180;
				osc_init.PLL.PLLP = RCC_PLLP_DIV2;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)							Error_Handler();

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

				if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK)		Error_Handler();

			break;
		case (uint32_t)50e3:

				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
				osc_init.HSEState = RCC_HSE_ON;
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
				osc_init.PLL.PLLM = 4;
				osc_init.PLL.PLLN = 50;
				osc_init.PLL.PLLP = RCC_PLLP_DIV2;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)							Error_Handler();

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

				if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_1) != HAL_OK)		Error_Handler();

			break;
		case (uint32_t)84e3:

				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
				osc_init.HSEState = RCC_HSE_ON;
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
				osc_init.PLL.PLLM = 4;
				osc_init.PLL.PLLN = 84;
				osc_init.PLL.PLLP = RCC_PLLP_DIV2;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)							Error_Handler();

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

				if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2) != HAL_OK)		Error_Handler();

			break;
		case (uint32_t)120e3:

				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
				osc_init.HSEState = RCC_HSE_ON;
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
				osc_init.PLL.PLLM = 4;
				osc_init.PLL.PLLN = 120;
				osc_init.PLL.PLLP = RCC_PLLP_DIV2;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)							Error_Handler();

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

				if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_3) != HAL_OK)		Error_Handler();

			break;
		default:
			break;
	}


	UART2_Init();
	RCC_Print_Clocks();

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

static void RCC_Print_Clocks(void){

	uint8_t msg[250];
	sprintf((char*)msg, "SYSCLOCK: %ld \n\r", HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&husart2, msg, strlen((char*) msg), HAL_MAX_DELAY);

	sprintf((char*)msg, "AHCLOCK: %ld \n\r", HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&husart2, msg, strlen((char*) msg), HAL_MAX_DELAY);

	sprintf((char*)msg, "PCLOCK 1: %ld \n\r", HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&husart2, msg, strlen((char*) msg), HAL_MAX_DELAY);

	sprintf((char*)msg, "PCLOCK 2: %ld \n\r", HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&husart2, msg, strlen((char*) msg), HAL_MAX_DELAY);

}

void Error_Handler(void){

	while(1);
}
