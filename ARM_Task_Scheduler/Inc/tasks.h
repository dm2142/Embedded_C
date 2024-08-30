/*
 * tasks.h
 *
 *  Created on: Apr 4, 2024
 *      Author: DM
 */

#ifndef TASKS_H_
#define TASKS_H_

#include <stdint.h>

/*
 * 	Define MACROS for memory sections for each Task and Scheduler Handler.
 */

#define MAX_TASKS				5
#define SIZE_TASK_STACK			1024U
#define SIZE_SCHEDULER_STACK	1024U

#define SRAM_START				0X20000000U								// Start address of SRAM.
#define SRAM_SIZE				( (128) * (1024) )						// 128 kB size
#define SRAM_END				( ( (SRAM_START) + (SRAM_SIZE) ))

#define TASK1_STACK_START		SRAM_END
#define TASK2_STACK_START		( (SRAM_END) - (1 * SIZE_TASK_STACK) )
#define TASK3_STACK_START		( (SRAM_END) - (2 * SIZE_TASK_STACK) )
#define TASK4_STACK_START		( (SRAM_END) - (3 * SIZE_TASK_STACK) )
#define IDLE_STACK_START		( (SRAM_END) - (4 * SIZE_TASK_STACK) )
#define SCHEDULER_STACK_START	( (SRAM_END) - (5 * SIZE_TASK_STACK) )

#define TASK_RUNNING_STATE		0x00
#define TASK_BLOCKED_STATE		0xFF

#define ORANGE_LED		0		// PC0
#define	GREEN_LED		1		// PC1
#define RED_LED			0		// PB0
#define YELLOW_LED		4		// PA4

#define DELAY_1MS		1250U
#define DELAY_1S		(1000 * (DELAY_1MS) )
#define DELAY_500MS		(500 * (DELAY_1MS) )
#define DELAY_250MS		(250 * (DELAY_1MS) )
#define DELAY_125MS		(125 * (DELAY_1MS) )

typedef struct task_control_block{
		uint32_t psp_value;
		uint32_t block_count;
		uint8_t current_state;
		void (*task_handler)(void);
}TCB_t;

void idle_task(void);

void task1_handler(void);

void task2_handler(void);

void task3_handler(void);

void task4_handler(void);

void task_delay(uint32_t tick_count);

void delay(uint32_t time_ms);

#endif /* TASKS_H_ */
