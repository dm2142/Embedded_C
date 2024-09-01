/*
 * ds1307.h
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include "stm32f446re.h"

/*
 *  Application configurable items
 */

#define DS1307_I2C			I2C3
#define DS1307_SCL_PORT		GPIOA
#define DS1307_SCL_PIN		8
#define DS1307_SDA_PORT		GPIOB
#define DS1307_SDA_PIN		4


/*
 * Register addresses
 */
#define DS1307_I2C_ADDR				0x68

#define DS1307_SECONDS_ADDR			0x00
#define DS1307_MINUTES_ADDR			0x01
#define DS1307_HOURS_ADDR			0x02
#define DS1307_DAY_ADDR				0x03
#define DS1307_DATE_ADDR			0x04
#define DS1307_MONTH_ADDR			0x05
#define DS1307_YEAR_ADDR			0x06
#define DS1307_CONTROL_ADDR			0x07

#define DS1307_TIMEFORMAT_12HRS		0x00
#define DS1307_TIMEFORMAT_24HRS		0x01

#define DS1307_TIMEFORMAT_12HRS_AM	0x00
#define	DS1307_TIMEFORMAT_12HRS_PM	0x01

typedef enum ds1307_days{
	SUNDAY = 0,
	MONDAY,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY,
}DS1307_Days_e;


/* *******************************************************
 * 					MAIN DS1307 STRUCTURE
 * *******************************************************
 */

typedef struct ds1307{
	struct ds1307_date{
		uint8_t	Seconds;
		uint8_t Minutes;
		uint8_t Hours;
	}Time;

	struct ds1307_time{
		uint8_t Day;
		uint8_t Date;
		uint8_t Month;
		uint8_t Year;
	}Date;
}DS1307_t;


/* ********************************************************
 * 					FUNCTION DECLARATION
 * ********************************************************
 */

uint8_t ds1307_init(void);

void ds1307_set_current_time(DS1307_t * ds1307_dev);
void ds1307_get_current_time(DS1307_t * ds1307_dev);

void ds1307_set_current_date(DS1307_t * ds1307_dev);
void ds1307_get_current_date(DS1307_t * ds1307_dev);

#endif /* INC_DS1307_H_ */

// Create a Configuration structure for initialization
//typedef struct ds1307_config{
//
//	struct {
//		uint8_t	u_seconds	: 4;
//		uint8_t d_seconds	: 3;
//		uint8_t clock_halt	: 1;
//	}Seconds;
//
//	struct {
//		uint8_t	u_minutes	: 4;
//		uint8_t d_minutes	: 3;
//	}Minutes;
//
//	struct {
//		uint8_t	u_hour		: 4;
//		uint8_t d_hour1		: 1;
//		uint8_t PM_AM_dhour2: 2;
//		uint8_t hourt_format: 1;
//	}Hours;
//
//	struct {
//		uint8_t u_date		: 4;
//		uint8_t d_date		: 2;
//	}Date;
//
//	struct {
//		uint8_t u_month		: 4;
//		uint8_t d_month		: 1;
//	}Month;
//
//	struct {
//		uint8_t u_date		: 4;
//		uint8_t d_date		: 4;
//	}Year;
//
//	struct {
//		uint8_t RS0			: 1;
//		uint8_t RS1			: 1;
//		uint8_t RESERVED1	: 2;
//		uint8_t SQWE		: 1;
//		uint8_t RESERVED2	: 2;
//		uint8_t OUT			: 1;
//	}Control;
//
//}DS1307_Config_t;


//typedef struct ds1307_date{
//	uint8_t	Seconds;
//	uint8_t Minutes;
//	uint8_t Hours;
//}DS1307_Date_t;
//
//typedef struct ds1307_time{
//	uint8_t Day;
//	uint8_t Date;
//	uint8_t Month;
//	uint8_t Year;
//}DS1307_Time_t;

//typedef struct ds1307_seconds{
//	uint8_t	u_seconds	: 4;
//	uint8_t d_seconds	: 3;
//	uint8_t clock_halt	: 1;
//}DS1307_Seconds_t;
//
//typedef struct ds1307_minutes{
//	uint8_t	u_minutes	: 4;
//	uint8_t d_minutes	: 3;
//}DS1307_Minutes_t;
//
//typedef struct ds1307_hours{
//	uint8_t	u_hour		: 4;
//	uint8_t d_hour1		: 1;
//	uint8_t PM_AM_dhour2: 2;
//	uint8_t hourt_format: 1;
//}DS1307_Hours_t;
//
//typedef struct ds1307_d{
//	uint8_t u_date		: 4;
//	uint8_t d_date		: 2;
//}DS1307_D_t;
//
//typedef struct ds1307_month{
//	uint8_t u_month		: 4;
//	uint8_t d_month		: 1;
//}DS1307_Month_t;
//
//typedef struct ds1307_year{
//	uint8_t u_date		: 4;
//	uint8_t d_date		: 4;
//}DS1307_Year_t;
//
//typedef struct ds1307_control{
//	uint8_t RS0			: 1;
//	uint8_t RS1			: 1;
//	uint8_t RESERVED1	: 2;
//	uint8_t SQWE		: 1;
//	uint8_t RESERVED2	: 2;
//	uint8_t OUT			: 1;
//}DS1307_Control_t;
