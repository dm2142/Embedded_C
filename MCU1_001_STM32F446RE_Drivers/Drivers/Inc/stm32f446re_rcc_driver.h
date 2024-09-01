/*
 * stm32f446re_rcc_driver.h
 *
 *  Created on: Apr 25, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_RCC_DRIVER_H_
#define INC_STM32F446RE_RCC_DRIVER_H_

#include "stm32f446re.h"

uint32_t GetAPB1Clock(void);
uint32_t GetAPB2Clock(void);

#endif /* INC_STM32F446RE_RCC_DRIVER_H_ */
