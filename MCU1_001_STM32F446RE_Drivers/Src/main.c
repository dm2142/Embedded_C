/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include <string.h>
#include "stm32f446re.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void SPI2_GPIOS_Config(void){

	GPIO_Handle_t gpios_port_b = {0};
	gpios_port_b.pGPIOx = GPIOB;
	gpios_port_b.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	gpios_port_b.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;
	gpios_port_b.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	gpios_port_b.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;
	gpios_port_b.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_5;

	// Configure SCK pin
	gpios_port_b.pGPIOx = GPIOB;
	gpios_port_b.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&gpios_port_b);

	// Configure MISO pin
	gpios_port_b.pGPIOx = GPIOB;
	gpios_port_b.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&gpios_port_b);

	// Configure MOSI pin
	gpios_port_b.pGPIOx = GPIOB;
	gpios_port_b.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&gpios_port_b);

	// Configure NSS pin
	gpios_port_b.pGPIOx = GPIOB;
	gpios_port_b.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&gpios_port_b);
}

void SPI2_Configuration(void){

	SPI_Handle_t p_spi2 = {0};
	p_spi2.pSPIx = SPI2;
	p_spi2.SPI_Congif.SPI_BusConfig = SPI_FULLDUPLEX;
	p_spi2.SPI_Congif.SPI_DataFormat = SPI_8_DATAFORMAT;
	p_spi2.SPI_Congif.SPI_DeviceMode = SPI_MASTER;
	p_spi2.SPI_Congif.SPI_FrameFormat = SPI_MSB_TRANSMIT_FIRST;
	p_spi2.SPI_Congif.SPI_SCKLSpeed = SPI_CLOCK_DIV_2;
	p_spi2.SPI_Congif.SPI_SSM = SPI_SSM_DISABLED;
	p_spi2.SPI_Congif.SPI_CPOL = SPI_CPOL_HIGH;
	p_spi2.SPI_Congif.SPI_CPHA = SPI_CPHA_FIRST_EDGE;

	SPI_Init(&p_spi2);
}

int main(void)
{

	GPIO_Handle_t user_btn = {
			.pGPIOx = GPIOC,
			.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13,
			.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT,
			.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED,
			.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD,
			.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL,
			.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0,
		};


	GPIO_Init(&led);
	GPIO_Init(&user_btn);

	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI10_15, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI10_15, 15);

	// SPI Test

	char * my_text = "Hello World from SPI Protocol";

	SPI2_GPIOS_Config();

	SPI2_Configuration();

	SPI2->CR2.SSOE = ENABLE;

	for(uint32_t i = 0; i < UINT16_MAX; i++);

	SPI2->CR1.SPE = ENABLE;
	SPI_SendData(SPI2, (uint8_t*) my_text, strlen(my_text));
	while(SPI2->SR.BSY == ENABLE);
	SPI2->CR1.SPE = DISABLE;


    /* Loop forever */
	for(;;){

//		if(GPIO_ReadFromInputPin(user_btn.pGPIOx, GPIO_PIN_13) == RESET){
//
//			GPIO_ToggleOutputPin(led.pGPIOx, GPIO_PIN_5);
//			for(uint32_t i = 0; i <= (UINT16_MAX*4); i++);
//
//		}

	}
}


void EXTI15_10_IRQHandler(void){

	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);

}
