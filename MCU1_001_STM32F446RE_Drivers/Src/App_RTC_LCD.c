/*
 * App_RTC_LCD.c
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446re.h"

#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLOCK 16000000
#define SYST_CSR				0xE000E010  // Privileged  SysTick Control and Status Register
#define SYST_RVR 				0xE000E014	// Privileged  SysTick Reload Value Register


void systick_timer_initialization(uint32_t tick_hz){

	// Calculate the system tick reload value
	uint32_t load_value = (SYSTICK_TIM_CLOCK / tick_hz) - 1;

	// Clear system tick Reload Value register (24 bit system timer)
	uint32_t * pSys_RVR = (uint32_t*) SYST_RVR;
	*pSys_RVR &= ~(0x00FFFFFF);

	// Load the start value to count.
	*pSys_RVR |= load_value;

	// Configure the SysTick Control and Status Register
	uint32_t * pSys_CSR = (uint32_t*) SYST_CSR;

	*pSys_CSR |= (1 << 2);	// Indicates the clock source: Processor clock source.
	*pSys_CSR |= (1 << 1);	// Enables SysTick exception request.
	*pSys_CSR |= (1 << 0);	// Enables the counter.
}

void number_to_string(uint8_t num , char* buf)
{

	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48;
		buf[1]= (num % 10) + 48;
	}
}

//hh:mm:ss
char* time_to_string(DS1307_t * rtc_dev)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_dev->Time.Hours,buf);
	number_to_string(rtc_dev->Time.Minutes,&buf[3]);
	number_to_string(rtc_dev->Time.Seconds,&buf[6]);

	buf[8] = '\0';

	return buf;

}

DS1307_t rtc;

int main(void) {

	lcd_init();
	lcd_set_cursor(1, 3);
	lcd_print_string("Starting RTC");

	ds1307_init();
	rtc.Time.Seconds = 25;
	rtc.Time.Minutes = 25;
	rtc.Time.Hours = 16;

	rtc.Date.Day = MONDAY;
	rtc.Date.Date = 29;
	rtc.Date.Month = 4;
	rtc.Date.Year = 24;

	ds1307_set_current_date(&rtc);
	ds1307_set_current_time(&rtc);

	lcd_set_cursor(2, 4);
	lcd_print_string("Complete");

	systick_timer_initialization(1);

	while(1);

}

void SysTick_Handler(void){

	ds1307_get_current_time(&rtc);
	ds1307_get_current_date(&rtc);
//	printf("Time %02d:%02d:%02d \n", rtc.Time.Hours, rtc.Time.Minutes, rtc.Time.Seconds );
//	printf("Day %d : Date %02d/%02d/%02d \n", rtc.Date.Day, rtc.Date.Date, rtc.Date.Month, rtc.Date.Year );

	lcd_set_cursor(2, 4);
	lcd_print_string(time_to_string(&rtc));

}
