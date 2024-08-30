/*
 * tasks.c
 *
 *  Created on: Apr 4, 2024
 *      Author: DM
 */

#include "tasks.h"
#include "registers.h"
#include <stdbool.h>
#include <stdio.h>

/*
 * 	Define the User Tasks
 */

void task1_handler(void){

	// Configure GPIO to control YELLOW Led (PC0).

	// Enable Clock for GPIO C port
	RCC_AHB1ENR_t volatile * const rcc_enable_ahb1 = (RCC_AHB1ENR_t*) (RCC_REGISTER | RCC_AHB1ENR_OFFSET);
	rcc_enable_ahb1->GPIO_C_EN = 0x01;

	// Configure GPIO C0 as output.
	GPIOx_MODER_t volatile * const p_gpio_c_moder_register = 	(GPIOx_MODER_t*) (GPIO_C_REGISTER | GPIO_MODER_OFFSET);
	p_gpio_c_moder_register->MODER_0 = 0x01;
	GPIOx_ODR_t volatile * const p_gpio_c_odr_r = (GPIOx_ODR_t*) (GPIO_C_REGISTER | GPIO_ODR_OFFSET);
	p_gpio_c_odr_r->ODR_0 = 0;

	printf("Running TASK 1 \n");

	while(true){

		p_gpio_c_odr_r->ODR_0 ^= 0x01;
		task_delay(1000);

	}
}

void task2_handler(void){

	// Configure GPIO to control RED Led (PC1).

		// Enable Clock for GPIO C port
		RCC_AHB1ENR_t volatile * const rcc_enable_ahb1 = (RCC_AHB1ENR_t*) (RCC_REGISTER | RCC_AHB1ENR_OFFSET);
		rcc_enable_ahb1->GPIO_C_EN = 0x01;

		// Configure GPIO C1 as output.
		GPIOx_MODER_t volatile * const p_gpio_c_moder_register = 	(GPIOx_MODER_t*) (GPIO_C_REGISTER | GPIO_MODER_OFFSET);
		p_gpio_c_moder_register->MODER_1 = 0x01;
		GPIOx_ODR_t volatile * const p_gpio_c_odr_r = (GPIOx_ODR_t*) (GPIO_C_REGISTER | GPIO_ODR_OFFSET);
		p_gpio_c_odr_r->ODR_1 = 0;

		printf("Running TASK 2 \n");

		while(true){

			p_gpio_c_odr_r->ODR_1 ^= 0x01;
			task_delay(500);
		}
}

void task3_handler(void){

	// Configure GPIO to control GREEN Led (PB0).

		// Enable Clock for GPIO B port
		RCC_AHB1ENR_t volatile * const rcc_enable_ahb1 = (RCC_AHB1ENR_t*) (RCC_REGISTER | RCC_AHB1ENR_OFFSET);
		rcc_enable_ahb1->GPIO_B_EN = 0x01;

		// Configure GPIO B0 as output.
		GPIOx_MODER_t volatile * const p_gpio_b_moder_register = 	(GPIOx_MODER_t*) (GPIO_B_REGISTER | GPIO_MODER_OFFSET);
		p_gpio_b_moder_register->MODER_0 = 0x01;
		GPIOx_ODR_t volatile * const p_gpio_b_odr_r = (GPIOx_ODR_t*) (GPIO_B_REGISTER | GPIO_ODR_OFFSET);
		p_gpio_b_odr_r->ODR_0 = 0;

		printf("Running TASK 3 \n");

		while(true){

			p_gpio_b_odr_r->ODR_0 ^= 0x01;
			task_delay(250);

		}
}

void task4_handler(void){

	// Configure GPIO to control ORANGE Led (PA4).

		// Enable Clock for GPIO A port
		RCC_AHB1ENR_t volatile * const rcc_enable_ahb1 = (RCC_AHB1ENR_t*) (RCC_REGISTER | RCC_AHB1ENR_OFFSET);
		rcc_enable_ahb1->GPIO_A_EN = 0x01;

		// Configure GPIO A4 as output.
		GPIOx_MODER_t volatile * const p_gpio_a_moder_register = 	(GPIOx_MODER_t*) (GPIO_A_REGISTER | GPIO_MODER_OFFSET);
		p_gpio_a_moder_register->MODER_4 = 0x01;
		GPIOx_ODR_t volatile * const p_gpio_a_odr_r = (GPIOx_ODR_t*) (GPIO_A_REGISTER | GPIO_ODR_OFFSET);
		p_gpio_a_odr_r->ODR_4 = 0;

		printf("Running TASK 4 \n");

		while(true){

			p_gpio_a_odr_r->ODR_4 ^= 0x01;
			task_delay(125);

		}
}

void idle_task(void){

	while(true);
}

void task_delay(uint32_t tick_count){

	extern TCB_t user_tasks[MAX_TASKS];
	extern uint32_t g_tick_count;
	extern uint8_t current_task;

	if( current_task > 0){

		user_tasks[current_task].block_count = g_tick_count + tick_count;
		user_tasks[current_task].current_state = TASK_BLOCKED_STATE;

		// Set PendSV Handler
		uint32_t* pICSR = (uint32_t*) 0xE000ED04;
		*pICSR |= (0x01 << 28);\

	}

}

void delay(uint32_t time_ms){

	for(volatile uint32_t load_value = 0; load_value < time_ms; load_value ++);

}
