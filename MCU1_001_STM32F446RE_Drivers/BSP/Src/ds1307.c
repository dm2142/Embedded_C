/*
 * ds13078.c
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */

#include "ds1307.h"

static I2C_Handle_t ds1307_i2c_handle = {0};

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);

static void ds1307_write(uint8_t data, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);

uint8_t ds1307_init(void){

	uint8_t clock_halt = 0;

	ds1307_i2c_pin_config();
	ds1307_i2c_config();

	I2C_EnablePer(DS1307_I2C, ENABLE);

	ds1307_write(0x00, DS1307_SECONDS_ADDR);
	clock_halt = ds1307_read(DS1307_SECONDS_ADDR);

	return ( (clock_halt >> 7) & 0x01 );
}

void ds1307_set_current_time(DS1307_t * ds1307_dev){

	// Set the 24 hrs time format as default.
	uint8_t aux1 = 0, aux2 = 0;

	aux2 = ds1307_dev->Time.Seconds / 10;
	aux1 = ( ( aux2 & 0b111 ) << 4 ) | ( ( ds1307_dev->Time.Seconds % 10 ) & 0b1111 );
	aux1 &= ~(0x1 << 7);
	ds1307_write(aux1, DS1307_SECONDS_ADDR);

	aux1 = 0;
	aux2 = ds1307_dev->Time.Minutes / 10;
	aux1 = ( ( aux2 & 0b111 ) << 4 ) | ( ( ds1307_dev->Time.Minutes % 10 ) & 0b1111 );
	ds1307_write(aux1, DS1307_MINUTES_ADDR);

	aux1 = 0;
	aux2 = ds1307_dev->Time.Hours / 10;
	aux1 = ( ( aux2 & 0b11 ) << 4 ) | ( ( ds1307_dev->Time.Hours % 10 ) & 0b1111 );
	ds1307_write(aux1, DS1307_HOURS_ADDR);
}

void ds1307_get_current_time(DS1307_t * ds1307_dev){

	uint8_t aux;
	aux = ds1307_read(DS1307_SECONDS_ADDR);
	ds1307_dev->Time.Seconds = (10 * ( (aux >> 4) & 0b111 )) + (aux & 0b1111);

	aux = ds1307_read(DS1307_MINUTES_ADDR);
	ds1307_dev->Time.Minutes = (10 * ( (aux >> 4) & 0b111 )) + (aux & 0b1111);

	aux = ds1307_read(DS1307_HOURS_ADDR);
	if( ( (aux >> 6) & 0x1 ) ){
		ds1307_dev->Time.Hours = (10 * ( (aux >> 4) & 0b1 )) + (aux & 0b1111);		// For 12 hrs format
	}else{
		ds1307_dev->Time.Hours = (10 * ( (aux >> 4) & 0b11 )) + (aux & 0b1111);		// For 24 hrs format
	}

}

void ds1307_set_current_date(DS1307_t * ds1307_dev){

	uint8_t aux1 = 0, aux2 = 0;

	aux1 = 0;
	aux2 = (ds1307_dev->Date.Date / 10) & 0b11;
	aux1 = ( ( aux2 & 0b11 ) << 4 ) | ( ( ds1307_dev->Date.Date % 10 ) & 0b1111 );
	ds1307_write(aux1, DS1307_DATE_ADDR);

	aux1 = (ds1307_dev->Date.Day);
	ds1307_write(aux1, DS1307_DAY_ADDR);

	aux1 = 0;
	aux2 = ds1307_dev->Date.Month / 10;
	aux1 = ( ( aux2 & 0b1 ) << 4 ) | ( ( ds1307_dev->Date.Month % 10 ) & 0b1111 );
	ds1307_write(aux1, DS1307_MONTH_ADDR);

	aux1 = 0;
	aux2 = ds1307_dev->Date.Year / 10;
	aux1 = ( ( aux2 & 0b1111 ) << 4 ) | ( ( ds1307_dev->Date.Year % 10 ) & 0b1111 );
	ds1307_write(aux1, DS1307_YEAR_ADDR);
}

void ds1307_get_current_date(DS1307_t * ds1307_dev){

	uint8_t aux;

	aux = ds1307_read(DS1307_DAY_ADDR);
	ds1307_dev->Date.Day = aux;

	aux = ds1307_read(DS1307_DATE_ADDR);
	ds1307_dev->Date.Date = (10 * ( (aux >> 4) & 0b11 )) + (aux & 0b1111);

	aux = ds1307_read(DS1307_MONTH_ADDR);
	ds1307_dev->Date.Month = (10 * ( (aux >> 4) & 0b1 )) + (aux & 0b1111);

	aux = ds1307_read(DS1307_YEAR_ADDR);
	ds1307_dev->Date.Year = (10 * ( (aux >> 4) & 0b1111 )) + (aux & 0b1111);
}


static void ds1307_i2c_pin_config(void){

	GPIO_Handle_t sda_pin = {0}, scl_pin = {0};

	sda_pin.pGPIOx = DS1307_SDA_PORT;
	sda_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	sda_pin.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_4;
	sda_pin.GPIO_PinConfig.GPIO_PinNumber = DS1307_SDA_PIN;
	sda_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPEN_DRAIN;
	sda_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	sda_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	scl_pin.pGPIOx = DS1307_SCL_PORT;
	scl_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FUN;
	scl_pin.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_4;
	scl_pin.GPIO_PinConfig.GPIO_PinNumber = DS1307_SCL_PIN;
	scl_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPEN_DRAIN;
	scl_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_NO_PD;
	scl_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	GPIO_Init(&sda_pin);
	GPIO_Init(&scl_pin);

}

static void ds1307_i2c_config(void){

	ds1307_i2c_handle.pI2Cx = DS1307_I2C;
	ds1307_i2c_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	ds1307_i2c_handle.I2C_Config.I2C_DeviceAddress = 0x0F;
	ds1307_i2c_handle.I2C_Config.I2C_FMDutyCycle = I2C_FMDUTY_2;
	ds1307_i2c_handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_STANDARD;

	I2C_Init(&ds1307_i2c_handle);
}

static void ds1307_write(uint8_t data, uint8_t reg_addr){

	uint8_t aux[2] = {0};
	aux[0] = reg_addr;
	aux[1] = data;

	I2C_MasterSendData(DS1307_I2C, aux, 2, DS1307_I2C_ADDR, I2C_START_REPEAT_DISABLE);

}

static uint8_t ds1307_read(uint8_t reg_addr){

	uint8_t aux = 0;
	I2C_MasterSendData(DS1307_I2C, &reg_addr, 1, DS1307_I2C_ADDR, I2C_START_REPEAT_ENABLE);
	I2C_MasterReceiveData(DS1307_I2C, &aux, 1, DS1307_I2C_ADDR, I2C_START_REPEAT_DISABLE);

	return aux;
}
