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
#include <stdio.h>
#include <stdbool.h>

/*
 *  Define the Register Address
 */

#define RCC_REGISTER 			0x40023800
#define RCC_AHB1ENR_OFFSET 		0x00000030

#define GPIO_A_REGISTER 		0x40020000
#define GPIO_C_REGISTER			0x40020800

#define GPIO_MODER_OFFSET		0x00000000
#define GPIO_ODR_OFFSET			0x00000014
#define GPIO_IDR_OFFSET			0x00000010


/*
 * This code enables the AHP1 clock for GPIO port A (PA5) to turn ON a built-in led.
 * The build settings for this project set a level of optimization -O2
 */



#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{

	// Create a pointer to AHP1 Clock for port GPIO A.
	uint32_t volatile * const rcc_enable_ahb1 = (uint32_t*) (RCC_REGISTER | RCC_AHB1ENR_OFFSET);
	printf("The pointer for RCC_REGISTER address is: %p \n", rcc_enable_ahb1);

	// Enables the AHP1 Clock for PORT GPIO A.
	*rcc_enable_ahb1 |= 0x01;

	// Enables the AHP1 Clock for PORT GPIO C.
	*rcc_enable_ahb1 |= (0x01 << 2);

	// Create a pointer to GPIO A configuration.
	uint32_t volatile * const p_gpio_a_moder_register = (uint32_t*) (GPIO_A_REGISTER | GPIO_MODER_OFFSET);
	uint32_t volatile * const p_gpio_a_output_data_register = (uint32_t*) (GPIO_A_REGISTER | GPIO_ODR_OFFSET);

	// Set port A pin 5 as output.
	*p_gpio_a_moder_register |= (0x01 << 10); //0x400;

	// Create a pointer to GPIO C configuration.
	uint32_t volatile * const p_gpio_c_moder_register = (uint32_t*) (GPIO_C_REGISTER | GPIO_MODER_OFFSET);
	uint32_t const volatile * const p_gpio_c_input_data_register = (uint32_t*) (GPIO_C_REGISTER | GPIO_IDR_OFFSET);
	// Use type qualifier volatile to tells the compiler do not optimize this variable.


	// Set port C pin 4 (c4) as input
	*p_gpio_c_moder_register |= (0x01 << 8);

    /* Loop forever */
	while(true)
	{

		// Check state from PIN 4 in Port C.
		if( (*p_gpio_c_input_data_register >> 4) & 0x01 )
		{
			// Set LED in High State.
			*p_gpio_a_output_data_register |= (0x01 << 5);
		}
		else
		{
			// Set LED in Low State.
			*p_gpio_a_output_data_register &= ~(0x01 << 5);
		}

	}
}
