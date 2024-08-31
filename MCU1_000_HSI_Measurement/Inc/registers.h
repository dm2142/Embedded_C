/*
 * Registers.h
 *
 *  Created on: Apr 10, 2024
 *      Author: DM
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

/*
 * 	Define the related registers to configure the HSI Clock, PORT A and MCO1 for measurement.
 */

#include <stdint.h>

#define RCC_ADDRESS			0x40023800UL
#define	RCC_CFGR_OFFSET		0x08UL			// RCC clock configuration register
#define RCC_CR_OFFSET		0x00UL
#define RCC_CFGR_ADDRESS	(RCC_ADDRESS + RCC_CFGR_OFFSET)

#define RCC_AHB1ENR_OFFSET 		0x00000030UL

#define GPIO_A_REGISTER 		0x40020000UL
#define GPIO_B_REGISTER			0x40020400UL
#define GPIO_C_REGISTER			0x40020800UL

#define GPIO_MODER_OFFSET		0x00000000UL
#define GPIO_ODR_OFFSET			0x00000014UL
#define GPIO_IDR_OFFSET			0x00000010UL
#define GPIO_PUPDR_OFFSET		0x0000000CUL
#define GPIO_AFRH_OFFSET		0x000000024L
#define GPIO_OSPEEDR_OFFSET		0x000000008L


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

typedef struct grpiox_ospeedr{

	uint32_t ospeed_0	: 2;
	uint32_t ospeed_1	: 2;
	uint32_t ospeed_2	: 2;
	uint32_t ospeed_3	: 2;
	uint32_t ospeed_4	: 2;
	uint32_t ospeed_5	: 2;
	uint32_t ospeed_6	: 2;
	uint32_t ospeed_7	: 2;
	uint32_t ospeed_8	: 2;
	uint32_t ospeed_9	: 2;
	uint32_t ospeed_10	: 2;
	uint32_t ospeed_11	: 2;
	uint32_t ospeed_12	: 2;
	uint32_t ospeed_13	: 2;
	uint32_t ospeed_14	: 2;
	uint32_t ospeed_15	: 2;

}  GPIOx_OSPEEDR_t;

typedef struct gpiox_afrh{

	uint32_t AFRH_8		: 4;
	uint32_t AFRH_9		: 4;
	uint32_t AFRH_10	: 4;
	uint32_t AFRH_11	: 4;
	uint32_t AFRH_12	: 4;
	uint32_t AFRH_13	: 4;
	uint32_t AFRH_14	: 4;
	uint32_t AFRH_15	: 4;

}GPIOx_AFRH_t;

typedef struct rcc_cr{

	uint32_t hsi_on		: 1;
	uint32_t hsi_rdy	: 1;
	uint32_t reserved_1	: 1;
	uint32_t hsi_trim	: 5;
	uint32_t hsi_cal	: 8;
	uint32_t hse_on		: 1;
	uint32_t hse_rdy	: 1;
	uint32_t hse_byp	: 1;
	uint32_t css_on		: 1;
	uint32_t reserved_2	: 4;
	uint32_t pll_on		: 1;
	uint32_t pll_rdy	: 1;
	uint32_t pll_i2s_on	: 1;
	uint32_t pll_i2s_rdy: 1;
	uint32_t pll_sai_on	: 1;
	uint32_t pll_sai_rdy: 1;
	uint32_t reserved_3	: 3;

}RCC_CR_t;

typedef struct clock_configuration_register {

	uint32_t sw 		: 2;
	uint32_t sws 		: 2;
	uint32_t h_pre 		: 4;
	uint32_t reserved_1 : 2;
	uint32_t p_pre_1 	: 3;
	uint32_t p_pre_2 	: 3;
	uint32_t rtc_pre	: 5;
	uint32_t mco_1		: 2;
	uint32_t reserved_2	: 1;
	uint32_t mco_1_pre	: 3;
	uint32_t mco_2_pre	: 3;
	uint32_t mco_2		: 2;

}RCC_CFGR_t;

#endif /* REGISTERS_H_ */
