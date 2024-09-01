/*
 * 013_I2C_Slave_Tx_Testing.c
 *
 *  Created on: Apr 24, 2024
 *      Author: DM
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446re.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


uint8_t command_code = 0;
char Data[255] = "Hello World from STM32F446\n";
I2C_Handle_t i2c = {0};

void I2C_GPIO_Config(void);
void I2C_Config(void);
void Button_Config(void);
void TestPin_Config(void);

int main(void)
{
	Button_Config();
	I2C_GPIO_Config();
	I2C_Config();
	I2C_IRQInterruptConfig(IRQ_NUM_I2C3_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NUM_I2C3_ER, ENABLE);

	I2C_IRQPriorityConfig(IRQ_NUM_I2C3_EV, 15);
	I2C_IRQPriorityConfig(IRQ_NUM_I2C3_EV, 14);

	i2c.pI2Cx->CR2.ITBUFEN = ENABLE;
	i2c.pI2Cx->CR2.ITERREN = ENABLE;
	i2c.pI2Cx->CR2.ITEVTEN = ENABLE;

	I2C3->CR1.PE = ENABLE;
	I2C3->CR1.ACK = ENABLE;

	while(1);

}

void I2C_Config(void){

	i2c.pI2Cx = I2C3;
	i2c.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	i2c.I2C_Config.I2C_DeviceAddress = 0x68;
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

void I2C3_EV_IRQHandler(void){
	I2C_EV_IRHandling(&i2c);
}


void I2C3_ER_IRQHandler(void){
	I2C_ER_IRHandling(&i2c);
}

void I2C_ApplicationEventCallback(I2C_Handle_t * pI2CHandle, uint8_t Event){

	static uint16_t idx = 0;

	if( Event == I2C_EVENTCALLBACK_DATA_REQUEST ){

		if( command_code == 0x51){
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen(Data));
		}else if( command_code == 0x52 ){
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Data[idx]);
			idx ++;
		}


	}else if( Event == I2C_EVENTCALLBACK_DATA_RECEIVE ){

		command_code = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if( Event == I2C_EVENTCALLBACK_ERROR_AF ){

		command_code = 0xFF;
		idx = 0;

	}else if( Event == I2C_EVENTCALLBACK_STOP ){

	}

}
