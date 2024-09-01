/*
 * 010_I2C_Tx_Testing.c
 *
 *  Created on: Apr 23, 2024
 *      Author: DM
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446re.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


char * my_text = "Hello World from I2C Protocol";

void I2C_GPIO_Config(void);
void I2C_Config(void);
void Button_Config(void);
void TestPin_Config(void);

int main(void)
{
	Button_Config();
	I2C_GPIO_Config();
	I2C_Config();

	while(1){

		if( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == RESET){

			for(uint32_t i = 0 ; i < 500000/2 ; i ++);
			I2C3->CR1.PE = ENABLE;
			I2C_MasterSendData(I2C3, (uint8_t*) my_text, strlen(my_text), 0x68);
			I2C3->CR1.PE = DISABLE;
		}


	}

}

void I2C_Config(void){

	I2C_Handle_t i2c = {0};
	i2c.pI2Cx = I2C3;
	i2c.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	i2c.I2C_Config.I2C_DeviceAddress = 0x61;
	i2c.I2C_Config.I2C_FMDutyCycle = I2C_FMDUTY_2;
	i2c.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STANDARD;

	I2C_Init(&i2c);

}

void I2C_GPIO_Config(void){

	GPIO_Handle_t i2c_pins = {0};

	i2c_pins.pGPIOx = GPIOB;
	i2c_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	i2c_pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_4;
	i2c_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	i2c_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPEN_DRAIN;
	i2c_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	// SCL pin
	i2c_pins.pGPIOx = GPIOA;
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_Init(&i2c_pins);

	// SDA pin
	i2c_pins.pGPIOx = GPIOB;
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_Init(&i2c_pins);

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

void TestPin_Config(void){

	GPIO_Handle_t test_pin = {0};

	test_pin.pGPIOx = GPIOB;
	test_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	test_pin.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0;
	test_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;
	test_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	test_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	test_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_11;
	GPIO_Init(&test_pin);

}
