/*
 * stm32f407xx.h
 *
 *  Created on: Oct 4, 2022
 *      Author: Eden
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/************* Base Addresses **************/

/* ARM CORTEX M4 - NVIC Base addresses */
#define NVIC_ISER0				( (__vo uint32_t*)0xE000E100 ) /* NVIC set enable register, IRQs 0 to 31    */
#define NVIC_ISER1				( (__vo uint32_t*)0xE000E104 ) /* NVIC set enable register, IRQs 32 to 63   */
#define NVIC_ISER2				( (__vo uint32_t*)0xE000E108 ) /* NVIC set enable register, IRQs 63 to 95   */
#define NVIC_ISER3				( (__vo uint32_t*)0xE000E10C ) /* NVIC set enable register, IRQs 96 to 127  */

#define NVIC_ICER0				( (__vo uint32_t*)0xE000E180 ) /* NVIC set disable register, IRQs 0 to 31    */
#define NVIC_ICER1				( (__vo uint32_t*)0xE000E184 ) /* NVIC set enable register,  IRQs 32 to 63   */
#define NVIC_ICER2				( (__vo uint32_t*)0xE000E188 ) /* NVIC set enable register,  IRQs 63 to 95   */
#define NVIC_ICER3				( (__vo uint32_t*)0xE000E18C ) /* NVIC set enable register,  IRQs 96 to 127  */

#define NVIC_PR_BASE_ADDR		( (__vo uint32_t*)0xE000E400 )

/* ARM Corterx M4 - Processor number of priority bits implements in PR */
#define NO_PR_BITS_IMPLEMENTED	4

/* Memory base addresses */
#define FLASH_BASEADDR			0x0800000U /* Flash memory base address */
#define SRAM1_BASEADDR			0x2000000U 	/* SRAM1 Base Address, 112KB */
#define SRAM2_BASEADDR			0x201C000U 	/* SRAM2 Base Address, 16KB */
#define ROM						0x1FFF0000U	/* System memory Base Address, 30KB */
#define SRAM					SRAM1_BASEADDR /* Main SRAM Base Address */

/* AHB and APB Bus peripheral base addresses */
#define PERPIHPERAL_BASE		0x40000000U
#define APB1PERIPH_BASE			PERPIHPERAL_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

/* Base addresses of AHB1 Bus peripherals */
#define GPIOA_BASEADDR		 	(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR 		 	(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		 	(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		 	(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		 	(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR 		 	(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR 		 	(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR 		 	(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR 		 	(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR 		 	(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR 		 	(AHB1PERIPH_BASE + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)

/* Base addresses of APB1 Bus peripherals */
#define I2C1_BASEADDR		 	(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		 	(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		 	(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

/* Base addresses of APB2 Bus peripherals */
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)



/**** Peripheral register structures ****/

/* GPIO_RegDef_t */
typedef struct
{
	__vo uint32_t MODER; 			/* GPIO port mode register */
	__vo uint32_t OTYPER; 			/* GPIO port output type register */
	__vo uint32_t OSPEEDR; 			/* GPIO port output speed register */
	__vo uint32_t PUPDR; 			/* GPIO port pull-up/pull-down register */
	__vo uint32_t IDR;				/* GPIO port input data register */
	__vo uint32_t ODR; 				/* GPIO port output data register */
	__vo uint32_t BSRR;				/* GPIO port bit set/reset register */
	__vo uint32_t LCKR; 				/* GPIO port configuration lock register */
	__vo uint32_t AFR[2];			/* GPIO alternate function */


}GPIO_RegDef_t;
/* RCC_RegDef_t */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t reserved4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t reserved[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


/* peripheral definitions */
#define GPIOA 			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI 			((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC 			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI 			((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG 			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1 			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2 			((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3 			((SPI_RegDef_t*) SPI3_BASEADDR)

/*********** ENABLE MACROS ***********/

/* Clock Enable macros for GPIOx peripherals */
#define GPIOx_PCLK_EN(x)	 (RCC->AHB1ENR |= (1 << x)) /* Enables Peripheral clock for given GPIO */
#define GPIOA_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 0)) /* Enables Peripheral clock for GPIOA */
#define GPIOB_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 1)) /* Enables Peripheral clock for GPIOB */
#define GPIOC_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 2)) /* Enables Peripheral clock for GPIOC */
#define GPIOD_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 3)) /* Enables Peripheral clock for GPIOD */
#define GPIOE_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 4)) /* Enables Peripheral clock for GPIOE */
#define GPIOF_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 5)) /* Enables Peripheral clock for GPIOF */
#define GPIOG_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 6)) /* Enables Peripheral clock for GPIOG */
#define GPIOH_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 7)) /* Enables Peripheral clock for GPIOH */
#define GPIOI_PCLK_EN()  	(RCC->AHB1ENR |= (1 << 8)) /* Enables Peripheral clock for GPIOI */

/* Clock Enable macros for SPI peripherals */
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 23))

/* Clock Enable macros for USART peripherals */

#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR |= (1 << 20))

#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= (1 << 5))

/* Clock Enable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))

/* Clock Enable macros for SPI peripherals */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12)) // on APB2 Bus
#define SPI2_PCLK_EN()	 	(RCC->APB1ENR |= (1 << 14)) // on APB1 Bus
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15)) // on APB1 Bus

/*********** DISABLE MACROS ***********/

/* Clock Disable macros for GPIOx peripherals */
#define GPIOx_PCLK_DI(x)	(RCC->AHB1ENR &= (0<< x)) /* Enables Peripheral clock for given GPIO */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= (0 << 0)) /* Enables Peripheral clock for GPIOA */
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= (0 << 1)) /* Enables Peripheral clock for GPIOB */
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= (0 << 2)) /* Enables Peripheral clock for GPIOC */
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= (0 << 3)) /* Enables Peripheral clock for GPIOD */
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= (0 << 4)) /* Enables Peripheral clock for GPIOE */
#define GPIOF_PCLK_DI()  	(RCC->AHB1ENR &= (0 << 5)) /* Enables Peripheral clock for GPIOF */
#define GPIOG_PCLK_DI()  	(RCC->AHB1ENR &= (0 << 6)) /* Enables Peripheral clock for GPIOG */
#define GPIOH_PCLK_DI()	  	(RCC->AHB1ENR &= (0 << 7)) /* Enables Peripheral clock for GPIOH */
#define GPIOI_PCLK_DI()  	(RCC->AHB1ENR &= (0 << 8)) /* Enables Peripheral clock for GPIOI */

/* Clock Disable macros for SPI peripherals */
#define I2C1_PCLK_DI() 		(RCC->APB1ENR &= (0 << 21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR &= (0 << 22))
#define I2C3_PCLK_DI() 		(RCC->APB1ENR &= (0 << 23))

/* Clock Disable macros for USART peripherals */

#define USART2_PCLK_DI() 	(RCC->APB1ENR &= (0 << 17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= (0 << 18))
#define UART4_PCLK_DI()  	(RCC->APB1ENR &= (0 << 19))
#define UART5_PCLK_DI()  	(RCC->APB1ENR &= (0 << 20))

#define USART1_PCLK_DI() 	(RCC->APB2ENR &= (0 << 4))
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= (0 << 5))

/* Clock Disable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= (0 << 14))


/* Clock Disable macros for SPI peripherals */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= (0 << 12)) // on APB2 Bus
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= (0 << 14)) // on APB1 Bus
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= (0 << 15)) // on APB1 Bus

/* GPIO To Number Macro */
#define GPIO_BASEADDR_TO_CODE(x)	  ( (x == GPIOA)? 0:\
										(x == GPIOB)? 1:\
										(x == GPIOC)? 2:\
										(x == GPIOD)? 3:\
										(x == GPIOE)? 4:\
										(x == GPIOF)? 5:\
										(x == GPIOG)? 6:\
										(x == GPIOH)? 7:\
										(x == GPIOI)? 8:0 )

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) //do while loop macro
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0) //do while loop macro
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0) //do while loop macro
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0) //do while loop macro
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0) //do while loop macro
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0) //do while loop macro
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0) //do while loop macro
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0) //do while loop macro
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0) //do while loop macro

/* EXTI IRQ Numbers */
#define IRQ_NO_EXTI_0		6
#define IRQ_NO_EXTI_1		7
#define IRQ_NO_EXTI_2		8
#define IRQ_NO_EXTI_3		9
#define IRQ_NO_EXTI_4		10
#define IRQ_NO_EXTI9_5		24
#define IRQ_NO_EXTI15_10	40

/* Generic Macros */
#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		DISABLE
#define HIGH				ENABLE
#define LOW					DISABLE
#define BTN_PRESSED			ENABLE

#endif /* INC_STM32F407XX_H_ */
