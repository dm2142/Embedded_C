/*
 * registers.h
 *
 *  Created on: Mar 19, 2024
 *      Author: DM
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

#include <stdint.h>

/*
 *  Define the Register Address
 */

#define RCC_REGISTER 			0x40023800
#define RCC_AHB1ENR_OFFSET 		0x00000030

#define GPIO_A_REGISTER 		0x40020000
#define GPIO_B_REGISTER			0x40020400
#define GPIO_C_REGISTER			0x40020800

#define GPIO_MODER_OFFSET		0x00000000
#define GPIO_ODR_OFFSET			0x00000014
#define GPIO_IDR_OFFSET			0x00000010
#define GPIO_PUPDR_OFFSET		0x0000000C


#define SYST_CSR				0xE000E010  // Privileged  SysTick Control and Status Register
#define SYST_RVR 				0xE000E014	// Privileged  SysTick Reload Value Register

#define SHCSR					0xE000ED24	// The SHCSR enables the system handlers, and indicates:the pending status of the BusFault, MemManage fault, and SVC exceptions

typedef struct rcc_ahb1enr {
	uint32_t GPIO_A_EN		:1;
	uint32_t GPIO_B_EN		:1;
	uint32_t GPIO_C_EN		:1;
	uint32_t GPIO_D_EN		:1;
	uint32_t GPIO_E_EN		:1;
	uint32_t GPIO_F_EN		:1;
	uint32_t GPIO_G_EN		:1;
	uint32_t GPIO_H_EN		:1;
	uint32_t reserved_1		:4;
	uint32_t CRC_EN			:1;
	uint32_t reserved_2		:5;
	uint32_t BKP_SRAM_EN	:1;
	uint32_t reserved_3		:2;
	uint32_t DMA_1_EN		:1;
	uint32_t DMA_2_EN		:1;
	uint32_t reserved_4		:6;
	uint32_t OTG_HS_EN		:1;
	uint32_t OTG_HS_ULP_EN	:1;
	uint32_t reserved_5		:1;
}RCC_AHB1ENR_t;


typedef struct gpiox_moder{
	uint32_t MODER_0	:2;
	uint32_t MODER_1	:2;
	uint32_t MODER_2	:2;
	uint32_t MODER_3	:2;
	uint32_t MODER_4	:2;
	uint32_t MODER_5	:2;
	uint32_t MODER_6	:2;
	uint32_t MODER_7	:2;
	uint32_t MODER_8	:2;
	uint32_t MODER_9	:2;
	uint32_t MODER_10	:2;
	uint32_t MODER_11	:2;
	uint32_t MODER_12	:2;
	uint32_t MODER_13	:2;
	uint32_t MODER_14	:2;
	uint32_t MODER_15	:2;
}GPIOx_MODER_t;

typedef struct gpiox_odr{
	uint32_t ODR_0		:1;
	uint32_t ODR_1		:1;
	uint32_t ODR_2		:1;
	uint32_t ODR_3		:1;
	uint32_t ODR_4		:1;
	uint32_t ODR_5		:1;
	uint32_t ODR_6		:1;
	uint32_t ODR_7		:1;
	uint32_t ODR_8		:1;
	uint32_t ODR_9		:1;
	uint32_t ODR_10		:1;
	uint32_t ODR_11		:1;
	uint32_t ODR_12		:1;
	uint32_t ODR_13		:1;
	uint32_t ODR_14		:1;
	uint32_t ODR_15		:1;
	uint32_t reserved	:16;
}GPIOx_ODR_t;

typedef struct gpiox_idr{
	uint32_t IDR_0		:1;
	uint32_t IDR_1		:1;
	uint32_t IDR_2		:1;
	uint32_t IDR_3		:1;
	uint32_t IDR_4		:1;
	uint32_t IDR_5		:1;
	uint32_t IDR_6		:1;
	uint32_t IDR_7		:1;
	uint32_t IDR_8		:1;
	uint32_t IDR_9		:1;
	uint32_t IDR_10		:1;
	uint32_t IDR_11		:1;
	uint32_t IDR_12		:1;
	uint32_t IDR_13		:1;
	uint32_t IDR_14		:1;
	uint32_t IDR_15		:1;
	uint32_t reserved	:16;
}GPIOx_IDR_t;

typedef struct gpiox_pupdr{
	uint32_t PUPDR_0	:2;
	uint32_t PUPDR_1	:2;
	uint32_t PUPDR_2	:2;
	uint32_t PUPDR_3	:2;
	uint32_t PUPDR_4	:2;
	uint32_t PUPDR_5	:2;
	uint32_t PUPDR_6	:2;
	uint32_t PUPDR_7	:2;
	uint32_t PUPDR_8	:2;
	uint32_t PUPDR_9	:2;
	uint32_t PUPDR_10	:2;
	uint32_t PUPDR_11	:2;
	uint32_t PUPDR_12	:2;
	uint32_t PUPDR_13	:2;
	uint32_t PUPDR_14	:2;
	uint32_t PUPDR_15	:2;
}GPIOx_PUPDR_t;

#endif /* REGISTERS_H_ */
