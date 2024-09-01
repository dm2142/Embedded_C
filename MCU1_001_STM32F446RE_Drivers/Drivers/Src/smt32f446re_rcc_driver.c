/*
 * smt32f446re_rcc_driver.c
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */


#include "stm32f446re_rcc_driver.h"

uint32_t GetAPB1Clock(void){

	uint32_t clock_freq, sys_clk;
	uint8_t aux, APB1_pres;
	uint16_t AHB_pres;
	const uint16_t AHB_pres_array[] = {2, 4, 8, 16, 64, 128, 256, 512};
	const uint16_t APB1_pres_array[] = {2, 4, 8, 16};

	aux = ( (RCC->CFGR >> 2) & 0b11 );	// Identify which is the clock source
	if( aux == 0b00 )		sys_clk = 16000000;	// HSI Oscillator set;
	else if( aux == 0b01 )	sys_clk =  8000000;	// HSE Oscillator set;
	else if( aux == 0b01 )	sys_clk =  0;	// PLL Oscillator set;
	else if( aux == 0b01 )	sys_clk =  0;	// PLL R Oscillator set;

	aux = ( (RCC->CFGR >> 4) & 0b1111 );
	if( aux < 8)	AHB_pres = 1;
	else			AHB_pres = AHB_pres_array[aux - 8];


	aux = ( (RCC->CFGR >> 10) & 0b111 );
	if( aux < 4)	APB1_pres = 1;
	else			APB1_pres = APB1_pres_array[aux - 4];

	clock_freq = ( (sys_clk / AHB_pres) / APB1_pres );

	return clock_freq;
}

uint32_t GetAPB2Clock(void){

	uint32_t clock_freq, sys_clk;
	uint8_t aux, APB2_pres;
	uint16_t AHB_pres;
	const uint16_t AHB_pres_array[] = {2, 4, 8, 16, 64, 128, 256, 512};
	const uint16_t APB2_pres_array[] = {2, 4, 8, 16};

	aux = ( (RCC->CFGR >> 2) & 0b11 );	// Identify which is the clock source
	if( aux == 0b00 )		sys_clk = 16000000;	// HSI Oscillator set;
	else if( aux == 0b01 )	sys_clk =  8000000;	// HSE Oscillator set;
	else if( aux == 0b01 )	sys_clk =  0;	// PLL Oscillator set;
	else if( aux == 0b01 )	sys_clk =  0;	// PLL R Oscillator set;

	aux = ( (RCC->CFGR >> 4) & 0b1111 );
	if( aux < 8)	AHB_pres = 1;
	else			AHB_pres = AHB_pres_array[aux - 8];


	aux = ( (RCC->CFGR >> 13) & 0b111 );
	if( aux < 4)	APB2_pres = 1;
	else			APB2_pres = APB2_pres_array[aux - 4];

	clock_freq = ( (sys_clk / AHB_pres) / APB2_pres );

	return clock_freq;
}
