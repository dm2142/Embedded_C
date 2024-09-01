/*
 * 008_SPI_cmd_handling.c
 *
 *  Created on: Apr 18, 2024
 *      Author: DM
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f446re.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define NACK 0xA5
#define ACK 0xF5

//command codes
#define COMMAND_LED_CTRL          	0x50
#define COMMAND_SENSOR_READ       	0x51
#define COMMAND_LED_READ         	0x52
#define COMMAND_PRINT           	0x53
#define COMMAND_ID_READ        	 	0x54

#define LED_ON      1
#define LED_OFF 	0

#define LED_PIN		9


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
	p_spi2.SPI_Congif.SPI_SCKLSpeed = SPI_CLOCK_DIV_8;
	p_spi2.SPI_Congif.SPI_SSM = SPI_SSM_DISABLED;
	p_spi2.SPI_Congif.SPI_CPOL = SPI_CPOL_LOW;
	p_spi2.SPI_Congif.SPI_CPHA = SPI_CPHA_FIRST_EDGE;

	SPI_Init(&p_spi2);
}

char * my_text = "Hello World from SPI Protocol";

int main(void)
{

	GPIO_Handle_t user_btn = {
			.pGPIOx = GPIOC,
			.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13,
			.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT,
			.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED,
			.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD,
			.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL,
			.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0,
		};

	GPIO_Init(&user_btn);

	// SPI Test
	SPI2_GPIOS_Config();
	SPI2_Configuration();
	SPI2->CR2.SSOE = ENABLE;

	uint8_t dummy_data = 0xff, dummy_read, ack_byte = 0, args[2] = {0, 0};

    /* Loop forever */
	while(true){

		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == SET );

		for(uint32_t i = 0; i < UINT16_MAX; i++);


		SPI2->CR1.SPE = ENABLE;

		// Send Command Number 1
		uint8_t cmmd_code = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &cmmd_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		SPI_SendData(SPI2, &dummy_data, 1);
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if( ack_byte == ACK ){

			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2);

		}

		while(SPI2->SR.BSY == ENABLE);
		SPI2->CR1.SPE = DISABLE;


	}
}


