/*
 * stm32f446re.h
 *
 *  Created on: Apr 11, 2024
 *      Author: DM
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

#include <stdint.h>
#include <stdlib.h>

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET

/*
 *  *********************************************************************************
 *  ******************************  MEMORY ADDRESSES  *******************************
 *  *********************************************************************************
 */

/*
 *  Define MACROS related with processor registers.
 */

#define NVIC_ISER0          ( (volatile uint32_t*)0xE000E100 )		// For IRQ0  to IRQ31
#define NVIC_ISER1          ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (volatile uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)

/*
 * 	Define  MACROS related with memory address
 */

#define SRAM1_BASEADDR				0x20000000UL
#define SRAM2_BASEADDR				0x2001C000UL
#define SRAM_BASEADDR				SRAM1_BASEADDR
#define FLASH_BASEADDR				0x08000000UL

/*
 * 	Define AHBx and APBx base address
 */

#define APB1_BASEADDR				0x40000000UL
#define APB2_BASEADDR				0x40010000UL
#define AHB1_BASEADDR				0x40020000UL
#define AHB2_BASEADDR				0x50000000UL
#define AHB3_BASEADDR				0x60000000UL

/*
 * 	Define base address peripherals which are hanging on APB 1
 */

#define	TIM2_BASEADDR				0x40000000UL
#define TIM3_BASEADDR				0x40000400UL
#define TIM4_BASEADDR				0x40000800UL
#define TIM5_BASEADDR				0x40000C00UL
#define TIM6_BASEADDR				0x40001000UL
#define TIM7_BASEADDR				0x40001400UL
#define TIM12_BASEADDR				0x40001800UL
#define TIM13_BASEADDR				0x40001C00UL
#define TIM14_BASEADDR				0x40002000UL
#define RTC_BKP_BASEADDR			0x40002800UL
#define WWDG_BASEADDR				0x40002C00UL
#define IWDG_BASEADDR				0x40003000UL
#define SPI2_BASEADDR				0x40003800UL
#define I2S2_BASEADDR				0x40003800UL
#define SPI3_BASEADDR				0x40003C00UL
#define I2S3_BASEADDR				0x40003C00UL
#define SPDIFRX_BASEADDR			0x40004000UL
#define USART2_BASEADDR				0x40004400UL
#define USART3_BASEADDR				0x40004800UL
#define UART4_BASEADDR				0x40004C00UL
#define UART5_BASEADDR				0x40005000UL
#define I2C1_BASEADDR				0x40005400UL
#define I2C2_BASEADDR				0x40005800UL
#define I2C3_BASEADDR				0x40005C00UL
#define FMPI2C1_BASEADDR			0x40006000UL
#define CAN1_BASEADDR				0x40006400UL
#define CAN2_BASEADDR				0x40006800UL
#define HDMI_CEC_BASEADDR			0x40006C00UL
#define PWR_BASEADDR				0x40007000UL
#define DAC_BASEADDR				0x40007400UL

/*
 * 	Define base address peripherals which are hanging on APB 2
 */

#define TIM1_BASEADDR				0x40010000UL
#define TIM8_BASEADDR				0x40010400UL
#define USART1_BASEADDR				0x40011000UL
#define USART6_BASEADDR				0x40011400UL
#define ADC123_BASEADDR				0x40012000UL
#define SDIO_BASEADDR				0x40012C00UL
#define SPI1_BASEADDR				0x40013000UL
#define SPI4_BASEADDR				0x40013400UL
#define SYSCFG_BASEADDR				0x40013800UL
#define EXTI_BASEADDR				0x40013C00UL
#define TIM9_BASEADDR				0x40014000UL
#define TIM10_BASEADDR				0x40014400UL
#define TIM11_BASEADDR				0x40014800UL
#define SAI1_BASEADDR				0x40015800UL
#define SAI2_BASEADDR				0x40015C00UL

/*
 * 	Define base address peripherals which are hanging on AHB 1
 */

#define GPIOA_BASEADDR				0x40020000UL
#define GPIOB_BASEADDR				0x40020400UL
#define GPIOC_BASEADDR				0x40020800UL
#define GPIOD_BASEADDR				0x40020C00UL
#define GPIOE_BASEADDR				0x40021000UL
#define GPIOF_BASEADDR				0x40021400UL
#define GPIOG_BASEADDR				0x40021800UL
#define GPIOH_BASEADDR				0x40021C00UL
#define CRC_BASEADDR				0x40023000UL
#define RCC_BASEADDR				0x40023800UL
#define	FLASH_INTERFACE_BASEADDR	0x40023C00UL
#define BKPSRAM_BASEADDR			0x40024000UL
#define DMA1_BASEADDR				0x40026000UL
#define DMA2_BASEADDR				0x40026400UL
#define USB_OTG_HS_BASEADDR			0x40040000UL

/*
 * 	Define base address peripherals which are hanging on AHB 2
 */

#define	USB_OTG_FS_BASEADDR			0x50000000UL
#define DCMI_BASEADDR				0x50050000UL

/*
 * 	Define base address peripherals which are hanging on AHB 3
 */

#define FMC1_BASEADDR				0x60000000UL
#define FMC3_BASEADDR				0x80000000UL
#define QSPI_BASEADDR				0x90000000UL
#define	FMC_CONTROL_BASEADDR		0xA0000000UL
#define	QSPI_CONTROL_BASEADDR		0xA0001000UL
#define FMC5_BASEADDR				0xC0000000UL
#define FMC6_BASEADDR				0xD0000000UL


/*
 * 	*********************************************************************************
 * 	************************* IRQ NUMBERS DEFINITIONS *******************************
 * 	*********************************************************************************
 */

#define IRQ_NUM_EXTI0  				6
#define IRQ_NUM_EXTI1  				7
#define IRQ_NUM_EXTI2  				8
#define IRQ_NUM_EXTI3  				9
#define IRQ_NUM_EXTI4  				10
#define IRQ_NUM_EXTI5_9  			23
#define IRQ_NUM_EXTI10_15  			40

#define IRQ_NUM_SPI1				35
#define IRQ_NUM_SPI2				36
#define IRQ_NUM_SPI3				51
#define IRQ_NUM_SPI4				84

#define IRQ_NUM_I2C1_EV				31
#define IRQ_NUM_I2C1_ER				32
#define IRQ_NUM_I2C2_EV				33
#define IRQ_NUM_I2C2_ER				34
#define IRQ_NUM_I2C3_EV				72
#define IRQ_NUM_I2C3_ER				73

#define IRQ_NUM_USART1				37
#define IRQ_NUM_USART2				38
#define IRQ_NUM_USART3				39
#define IRQ_NUM_UART4				52
#define IRQ_NUM_UART5				53
#define IRQ_NUM_USART6				71
/*
 * 	*********************************************************************************
 * 	*****************  STRUCTURES DEFINITION FOR PERIPHERALS ************************
 * 	*********************************************************************************
 */

// Peripheral structure for GPIO
typedef struct{

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEED;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;


typedef struct{

	volatile struct {
		uint32_t CPHA		: 1;
		uint32_t CPOL		: 1;
		uint32_t MSTR		: 1;
		uint32_t BR			: 3;
		uint32_t SPE		: 1;
		uint32_t LSBFIRST	: 1;
		uint32_t SSI		: 1;
		uint32_t SSM		: 1;
		uint32_t RXONLY		: 1;
		uint32_t DFF		: 1;
		uint32_t CRCNEXT	: 1;
		uint32_t CRCEN		: 1;
		uint32_t BIDIOE		: 1;
		uint32_t BIDIMODE	: 1;
	}CR1;

	volatile struct {
		uint32_t RXDMAEN	: 1;
		uint32_t TXDMAEN	: 1;
		uint32_t SSOE		: 1;
		uint32_t RESERVED	: 1;
		uint32_t FRF		: 1;
		uint32_t ERRIE		: 1;
		uint32_t RXNEIE		: 1;
		uint32_t TXEIE		: 1;
	}CR2;

	volatile struct {
		uint32_t RXNE		: 1;
		uint32_t TXE		: 1;
		uint32_t CHSIDE		: 1;
		uint32_t UDR		: 1;
		uint32_t CRCERR		: 1;
		uint32_t MODF		: 1;
		uint32_t OVR		: 1;
		uint32_t BSY		: 1;
		uint32_t FRE		: 1;
	}SR;

	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;

	volatile struct {
		uint32_t CHLEN		: 1;
		uint32_t DATLEN		: 1;
		uint32_t CKPOL		: 1;
		uint32_t I2SSTD		: 1;
		uint32_t RESERVED	: 1;
		uint32_t PCMSYNC	: 1;
		uint32_t I2SCFG		: 1;
		uint32_t I2SE		: 1;
		uint32_t I2SMOD		: 1;
		uint32_t ASTREN		: 1;
	}I2SCFGR;

	volatile struct {
		uint32_t I2SDIV		: 8;
		uint32_t ODD		: 1;
		uint32_t MCKOE		: 1;
	}I2SSPR;

}SPI_RegDef_t;

typedef struct{

	volatile struct{
		uint32_t PE			: 1;
		uint32_t SMBUS		: 1;
		uint32_t RESERVED1	: 1;
		uint32_t SMBTYPE	: 1;
		uint32_t ENARP		: 1;
		uint32_t ENPEC		: 1;
		uint32_t ENGC		: 1;
		uint32_t NOSTRETCH	: 1;
		uint32_t START		: 1;
		uint32_t STOP		: 1;
		uint32_t ACK		: 1;
		uint32_t POS		: 1;
		uint32_t PEC		: 1;
		uint32_t ALERT		: 1;
		uint32_t RESERVED2	: 1;
		uint32_t SWRST		: 1;
	}CR1;

	volatile struct{
		uint32_t FREQ		: 6;
		uint32_t RESERVED1	: 2;
		uint32_t ITERREN	: 1;
		uint32_t ITEVTEN	: 1;
		uint32_t ITBUFEN	: 1;
		uint32_t DMAEN		: 1;
		uint32_t LAST		: 1;
	}CR2;

	volatile struct{
		uint32_t ADD0		: 1;
		uint32_t ADD1		: 7;
		uint32_t ADD2		: 2;
		uint32_t RESERVED	: 4;
		uint32_t HIGH_STATE	: 1;
		uint32_t ADDMODE	: 1;
	}OAR1;

	volatile struct{
		uint32_t ENDUAL		: 2;
		uint32_t ADD		: 7;
	}OAR2;

	volatile uint32_t DR;

	volatile struct{
		uint32_t SB			: 1;
		uint32_t ADDR		: 1;
		uint32_t BTF		: 1;
		uint32_t ADD10		: 1;
		uint32_t STOPBF		: 1;
		uint32_t RESERVED1	: 1;
		uint32_t RXNE		: 1;
		uint32_t TXE		: 1;
		uint32_t BERR		: 1;
		uint32_t ARLO 		: 1;
		uint32_t AF			: 1;
		uint32_t OVR		: 1;
		uint32_t PECERR		: 1;
		uint32_t RESERVED2	: 1;
		uint32_t TIMEOUT	: 1;
		uint32_t SMBALERT	: 1;
	}SR1;

	volatile struct{
		uint32_t MSL		: 1;
		uint32_t BUSY		: 1;
		uint32_t TRA		: 1;
		uint32_t RESERVED	: 1;
		uint32_t GENCALL	: 1;
		uint32_t SMBDEFAULT	: 1;
		uint32_t SMBHOST	: 1;
		uint32_t DUALF		: 1;
		uint32_t PEC		: 8;
	}SR2;

	volatile struct{
		uint32_t CCR		: 12;
		uint32_t RESERVED	: 2;
		uint32_t DUTY		: 1;
		uint32_t FS			: 1;
	}CCR;

	volatile uint32_t TRISE;

	volatile struct{
		uint32_t DNF		: 4;
		uint32_t ANOFF		: 1;
	}FLTR;

}I2C_RegDef_t;

typedef struct usart_regdef{

	volatile struct{
		uint32_t PE		: 1;
		uint32_t FE		: 1;
		uint32_t NF		: 1;
		uint32_t ORE	: 1;
		uint32_t IDLE	: 1;
		uint32_t RXNE	: 1;
		uint32_t TC		: 1;
		uint32_t TXE	: 1;
		uint32_t LBD	: 1;
		uint32_t CTS	: 1;
	}SR;

	uint32_t DR;

	volatile struct{
		uint32_t DIV_FRACTION	: 4;
		uint32_t DIV_MANTISSA	: 12;
	}BRR;

	volatile struct{
		uint32_t SBK		: 1;
		uint32_t RWU		: 1;
		uint32_t RE			: 1;
		uint32_t TE			: 1;
		uint32_t IDLEIE		: 1;
		uint32_t RXNEIE		: 1;
		uint32_t TCIE		: 1;
		uint32_t TXEIE		: 1;
		uint32_t PEIE		: 1;
		uint32_t PS			: 1;
		uint32_t PCE		: 1;
		uint32_t WAKE		: 1;
		uint32_t M			: 1;
		uint32_t UE			: 1;
		uint32_t RESERVED	: 1;
		uint32_t OVER8		: 1;
	}CR1;

	volatile struct{
		uint32_t ADD		: 4;
		uint32_t RESERVED1	: 1;
		uint32_t LBDL		: 1;
		uint32_t LBDIE		: 1;
		uint32_t RESERVED2	: 1;
		uint32_t LBCL		: 1;
		uint32_t CPHA		: 1;
		uint32_t CPOL		: 1;
		uint32_t CLKEN		: 1;
		uint32_t STOP		: 2;
		uint32_t LINEN		: 1;
	}CR2;

	volatile struct{
		uint32_t EIE		: 1;
		uint32_t IREN		: 1;
		uint32_t IRLP		: 1;
		uint32_t HDSEL		: 1;
		uint32_t NACK		: 1;
		uint32_t SCEN		: 1;
		uint32_t DMAR		: 1;
		uint32_t DMAT		: 1;
		uint32_t RTSE		: 1;
		uint32_t CTSE		: 1;
		uint32_t CTSIE		: 1;
		uint32_t ONEBI		: 1;
	}CR3;

	volatile struct{
		uint32_t PSC		: 8;
		uint32_t GT		: 8;
	}GTPR;

}USART_RegDef_t;

// Peripheral structure for RCC (Reset and Clock Control)
typedef struct{

	volatile uint32_t CR;
	volatile uint32_t PLL_CFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1_RSTR;
	volatile uint32_t AHB2_RSTR;
	volatile uint32_t AHB3_RSTR;
	const uint32_t RESERVED_1;
	volatile uint32_t APB1_RSTR;
	volatile uint32_t APB2_RSTR;
	const uint32_t RESERVED_2 [2];
	volatile uint32_t AHB1_ENR;
	volatile uint32_t AHB2_ENR;
	volatile uint32_t AHB3_ENR;
	const uint32_t RESERVED_3;
	volatile uint32_t APB1_ENR;
	volatile uint32_t APB2_ENR;
	const uint32_t RESERVED_4[2];
	volatile uint32_t AHB1_LPENR;
	volatile uint32_t AHB2_LPENR;
	volatile uint32_t AHB3_LPENR;
	const uint32_t RESERVED_5;
	volatile uint32_t APB1_LPENR;
	volatile uint32_t APB2_LPENR;
	const uint32_t RESERVED_6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	const uint32_t RESERVED_7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2_SCFGR;
	volatile uint32_t PLLSAI_CFGR;
	volatile uint32_t DCK_CFGR;
	volatile uint32_t CKGAT_ENR;
	volatile uint32_t DCK_CFGR2;

}RCC_RegDef_t;

typedef struct {

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;

typedef struct {

	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED_1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t RESERVED_2[2];
	volatile uint32_t CFGR;

}SYSCFG_RegDef_t;

/*
 *  *********************************************************************************
 * 	**************** PERIPHERALS DEFINITIONS (TYPECASTED MACROS) ********************
 * 	*********************************************************************************
 */

#define GPIOA					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define SPI1					((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1					((USART_RegDef_t*) USART1_BASEADDR)
#define USART2					((USART_RegDef_t*) USART2_BASEADDR)
#define USART3					((USART_RegDef_t*) USART3_BASEADDR)
#define UART4					((USART_RegDef_t*) UART4_BASEADDR)
#define UART5					((USART_RegDef_t*) UART5_BASEADDR)
#define USART6					((USART_RegDef_t*) USART6_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


/*
 * 	*********************************************************************************
 * 	************************** FUNCTIONS LIKE MACROS ********************************
 * 	*********************************************************************************
 */

// Enable clock peripheral for GPIOx peripherals
#define GPIOA_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 0))
#define GPIOB_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 1))
#define GPIOC_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 2))
#define GPIOD_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 3))
#define GPIOE_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 4))
#define GPIOF_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 5))
#define GPIOG_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 6))
#define GPIOH_CLOCK_EN()		(RCC->AHB1_ENR |= (1 << 7))

// Disable clock peripheral for GPIOx peripherals
#define GPIOA_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 0))
#define GPIOB_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 1))
#define GPIOC_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 2))
#define GPIOD_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 3))
#define GPIOE_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 4))
#define GPIOF_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 5))
#define GPIOG_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 6))
#define GPIOH_CLOCK_DIS()		(RCC->AHB1_ENR &= ~(1 << 7))

//	Enable clock peripheral for SPIx
#define SPI1_CLOCK_EN()				(RCC->APB2_ENR |= (1 <<12))
#define SPI2_CLOCK_EN()				(RCC->APB1_ENR |= (1 <<14))
#define SPI3_CLOCK_EN()				(RCC->APB1_ENR |= (1 <<15))
#define SPI4_CLOCK_EN()				(RCC->APB2_ENR |= (1 <<13))

#define SPI1_CLOCK_DIS()			(RCC->APB2_ENR &= ~(1 <<12))
#define SPI2_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<14))
#define SPI3_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<15))
#define SPI4_CLOCK_DIS()			(RCC->APB2_ENR &= ~(1 <<13))

// Enable clock peripheral for I2Cx
#define I2C1_CLOCK_EN()				(RCC->APB1_ENR |= (1 <<21))
#define I2C2_CLOCK_EN()				(RCC->APB1_ENR |= (1 <<22))
#define I2C3_CLOCK_EN()				(RCC->APB1_ENR |= (1 <<23))

#define I2C1_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<21))
#define I2C2_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<22))
#define I2C3_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<23))

// Enable clock peripheral for USARTx/UART
#define USART1_CLOCK_EN()			(RCC->APB2_ENR |= (1 <<4))
#define USART2_CLOCK_EN()			(RCC->APB1_ENR |= (1 <<17))
#define USART3_CLOCK_EN()			(RCC->APB1_ENR |= (1 <<18))
#define UART4_CLOCK_EN()			(RCC->APB1_ENR |= (1 <<19))
#define UART5_CLOCK_EN()			(RCC->APB1_ENR |= (1 <<20))
#define USART6_CLOCK_EN()			(RCC->APB2_ENR |= (1 <<5))

#define USART1_CLOCK_DIS()			(RCC->APB2_ENR &= ~(1 <<4))
#define USART2_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<17))
#define USART3_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<18))
#define UART4_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<19))
#define UART5_CLOCK_DIS()			(RCC->APB1_ENR &= ~(1 <<20))
#define USART6_CLOCK_DIS()			(RCC->APB2_ENR &= ~(1 <<5))

// Enable clock peripheral for system configuration controller (SYSCFG)
#define SYSCFG_CLOCK_EN()		(RCC->APB2_ENR |= (1 << 14))

// Reset peripheral register from RCC Register
#define GPIOA_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 0); RCC->AHB1_RSTR &= ~(1 << 0); }while(0)
#define GPIOB_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 1); RCC->AHB1_RSTR &= ~(1 << 1); }while(0)
#define GPIOC_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 2); RCC->AHB1_RSTR &= ~(1 << 2); }while(0)
#define GPIOD_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 3); RCC->AHB1_RSTR &= ~(1 << 3); }while(0)
#define GPIOE_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 4); RCC->AHB1_RSTR &= ~(1 << 4); }while(0)
#define GPIOF_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 5); RCC->AHB1_RSTR &= ~(1 << 5); }while(0)
#define GPIOG_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 6); RCC->AHB1_RSTR &= ~(1 << 6); }while(0)
#define GPIOH_REG_RST()			do{ RCC->AHB1_RSTR |= (1 << 7); RCC->AHB1_RSTR &= ~(1 << 7); }while(0)

// Reset the peripheral register for SPI from RCC
#define SPI1_REG_RST()			do{ RCC->APB2_RSTR |= (1 << 12); RCC->APB2_RSTR &= ~(1 << 12); }while(0)
#define SPI2_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 14); RCC->APB1_RSTR &= ~(1 << 14); }while(0)
#define SPI3_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 15); RCC->APB1_RSTR &= ~(1 << 15); }while(0)
#define SPI4_REG_RST()			do{ RCC->APB2_RSTR |= (1 << 13); RCC->APB2_RSTR &= ~(1 << 13); }while(0)

// Reset the peripheral register for I2C from RCC
#define I2C1_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 21); RCC->APB1_RSTR &= ~(1 << 21); }while(0)
#define I2C2_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 22); RCC->APB1_RSTR &= ~(1 << 22); }while(0)
#define I2C3_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 23); RCC->APB1_RSTR &= ~(1 << 23); }while(0)

#define USART1_REG_RST()			do{ RCC->APB2_RSTR |= (1 << 4); RCC->APB2_RSTR &= ~(1 << 4); }while(0)
#define USART2_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 17); RCC->APB1_RSTR &= ~(1 << 17); }while(0)
#define USART3_REG_RST()			do{ RCC->APB1_RSTR |= (1 << 18); RCC->APB1_RSTR &= ~(1 << 18); }while(0)
#define UART4_REG_RST()				do{ RCC->APB1_RSTR |= (1 << 19); RCC->APB1_RSTR &= ~(1 << 19); }while(0)
#define UART5_REG_RST()				do{ RCC->APB1_RSTR |= (1 << 20); RCC->APB1_RSTR &= ~(1 << 20); }while(0)
#define USART6_REG_RST()			do{ RCC->APB2_RSTR |= (1 << 5); RCC->APB2_RSTR &= ~(1 << 5); }while(0)

// This macro returns a code( between 0 to 7) for a given GPIO base address(x)
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7:0)

#include "stm32f446re_rcc_driver.h"
#include "stm32f446re_gpio_driver.h"
#include "stm32f446re_spi_driver.h"
#include "stm32f446re_i2c_driver.h"
#include "stm32f446re_usart_driver.h"


#endif /* INC_STM32F446RE_H_ */
