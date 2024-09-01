/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: Apr 13, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include "stm32f446re.h"

/*
 * ********************************************************
 * 				HANDLE STRUCTURES FOR GPIO
 * ********************************************************
 */

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t * pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * ********************************************************
 * 				FUNCTIONS DECLARATIONS FOR GPIO
 * ********************************************************
 */

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * ********************************************************
 * 				ENUMS FOR GPIO CONFIGURATION
 * ********************************************************
 */

/*
 * @ GPIO PIN NUMBERS
 */
typedef enum {
	GPIO_PIN_0 = 0,
	GPIO_PIN_1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
}GPIO_PIN_NO_e;


/*
 * @ GPIO MODES
 * Possible GPIO pin modes
 */
typedef enum {
	GPIO_MODE_INPUT = 0,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALT_FUN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,
	GPIO_MODE_IT_RT,
	GPIO_MODE_IT_RFT,
}GPIO_MODE_e;

/*
 * @ GPIO OUTPUT TYPES
 */
typedef enum {
	GPIO_PUSH_PULL,
	GPIO_OPEN_DRAIN,
}GPIO_OUTYPE_e;

/*
 * @ GPIO OUTPUT SPEED
 */
typedef enum {
	GPIO_LOW_SPEED,
	GPIO_MEDIUM_SPEED,
	GPIO_FAST_SPEED,
	GPIO_HIGH_SPEED,
}GPIO_OSPEED_e;

/*
 * @ GPIO PULL UP PULL DOWN
 */
typedef enum{
	GPIO_NO_PU_NO_PD,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
}GPIO_PUPD_e;

/*
 * @ ALTERNATE FUNCTION VALUES
 */
typedef enum{

	GPIO_AF_0,
	GPIO_AF_1,
	GPIO_AF_2,
	GPIO_AF_3,
	GPIO_AF_4,
	GPIO_AF_5,
	GPIO_AF_6,
	GPIO_AF_7,
	GPIO_AF_8,
	GPIO_AF_9,
	GPIO_AF_10,
	GPIO_AF_11,
	GPIO_AF_12,
	GPIO_AF_13,
	GPIO_AF_14,
	GPIO_AF_15,
}GPIO_ALTFUN_e;

#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
