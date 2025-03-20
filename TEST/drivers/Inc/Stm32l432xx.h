/*
 * Stm32l432xx.h
 *
 *  Created on: Mar 15, 2025
 *      Author: ADMIN
 */

#ifndef INC_STM32L432XX_H_
#define INC_STM32L432XX_H_

#include <stdint.h>
#include <stddef.h> //Null declaration

// Short form of volatile to be used widely
#define _vo volatile
#define __weak __attribute__((weak))

/*
 * PROCESSOR SPECIFIC DETAILS
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 * */
#define NVIC_ISER0 				((_vo uint32_t*)0xE000E100)		   //DUIP219
#define NVIC_ISER1 				((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2 				((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3 				((_vo uint32_t*)0xE000E10C)

/*
 * ARM Cortext Mx Processor NVIC ICERx register addresses
 * */
#define NVIC_ICER0 				((_vo uint32_t*)0xE000E180)		   //DUIP219
#define NVIC_ICER1 				((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2 				((_vo uint32_t*)0XE000E188)
#define NVIC_ICER3 				((_vo uint32_t*)0XE000E18C)


/*
 * ARM Cortext Mx Processor Priority register address Calculation
 * */
#define NVIC_PR_BASE_ADDR 		((_vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED  4
/*
 * Base addresses of Flash and SRAM memories
 * */
/* We could use the suffix DRV_FLASH means belonging to the driver layer
 * differentiate from the low-level layer, middleware layer
 * */
#define FLASH_BASEADDR	         0x08000000U        				//MaimemP117rm0432
#define SRAM1_BASEADDR	         0x20000000U						//P16DtSh
#define SRAM                     SRAM1_BASEADDR	    				//BCwui
#define SRAM2_BASEADDR			 0x10000000U        				//P16DtSh
#define ROM_BASEADDR			 0x1FFF0000U	    				//SysmemP117rm0432


/*
 * Base addresses of bus domains of the MCU
 * */
#define PERIPH_BASE 			0x40000000U							/*Assigned to APB1 as it explicitly contains general-purpose peripherals while AHB contains memory controllers and DMA controllers also*/
#define AHB1PERIPH_BASEADDR 	0x40020000U         				//P95rm0432 	/*DMA1:First peripheral from the bus and the reserved region before suggests no other AHB1 peripheral lower address*/
#define AHB2PERIPH_BASEADDR 	0x48000000U							//P95rm0432
#define APB1PERIPH_BASEADDR		PERIPH_BASE							//P95rm0432
#define APB2PERIPH_BASEADDR		0x40010000U							//P95rm0432

/*
 * Base addresses of peripherals hanging on the AHB1 bus
 * */

#define RCC_BASEADDR 			(AHB1PERIPH_BASEADDR+0x1000)



/*
 * Base addresses of peripherals hanging on the AHB2 bus
 * */
#define GPIOA_BASEADDR			(AHB2PERIPH_BASEADDR+0x0000)		//P97rm0432
#define GPIOB_BASEADDR			(AHB2PERIPH_BASEADDR+0x0400)		//AHB2 + offset
#define GPIOC_BASEADDR			(AHB2PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR			(AHB2PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR			(AHB2PERIPH_BASEADDR+0x1C00)
#define GPIOI_BASEADDR			(AHB2PERIPH_BASEADDR+0x2000)

/*
 * Base addresses of peripherals hanging on the APB1 bus
 * */

#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR+0x3800)		//P101rm0432
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR+0x3C00)		//APB1 + offset
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR+0x4800)
#define	UART4_BASEADDR			(APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR+0x5000)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR+0x5C00)
#define I2C4_BASEADDR			(APB1PERIPH_BASEADDR+0x8400)



/*
 * Base addresses of peripherals hanging on the APB2 bus
 * */

#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR+0x0000)		//P99rm0432
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR+0x0400)		//APB2 + offset
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR+0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR+0x3800)
/*
 * Possible SPI Application States
 * */
#define SPI_READY 				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/*
 * Structure for the GPIO peripheral register
 * */
typedef struct{
	//Volatile for updating every clock cycle (for instance)
	_vo uint32_t MODER;													//P342rm0432 //offset: 0x00
	_vo uint32_t OTYPER;												 			 //offset: 0x04
	_vo uint32_t OSPEEDR;															 //offset: 0x08
	_vo uint32_t PUPDR;																 //offset: 0x0C
	_vo uint32_t IDR;																 //offset: 0x10
	_vo uint32_t ODR;																 //offset: 0x14
	_vo uint32_t BSRR;																 //offset: 0x18
	_vo uint32_t LCKR;																 //offset: 0x1C
	_vo uint32_t AFR[2];															 //offset: 0x20 - 0x24
}GPIO_Reg_Def_t;

/*
 * Peripherals definitions (Peripheral base addresses typecasted to xxx_Reg_Def_t)
 * */
#define GPIOA 					((GPIO_Reg_Def_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_Reg_Def_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_Reg_Def_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_Reg_Def_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_Reg_Def_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_Reg_Def_t*)GPIOF_BASEADDR)
#define GPIOG 					((GPIO_Reg_Def_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_Reg_Def_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_Reg_Def_t*)GPIOI_BASEADDR)

/*
 * peripheral register definition structure for RCC
 * */
typedef struct{
	uint32_t RCC_CR;
	uint32_t RCC_ICSCR;
	uint32_t RCC_CFGR;
	uint32_t RCC_PLLCFGR;
	uint32_t RCC_PLLSAI1CFGR;
	uint32_t RCC_PLLSAI2CFGR;
	uint32_t RCC_CIER;
	uint32_t RCC_CIFR;
	uint32_t RCC_CICR;
	uint32_t RCC_AHB1RSTR;
	uint32_t RCC_AHB2RSTR;
	uint32_t RCC_AHB3RSTR;
	uint32_t RCC_APB1RSTR1;
	uint32_t RCC_APB1RSTR2;
	uint32_t RCC_APB2RSTR;
	uint32_t RCC_AHB1ENR;
	uint32_t RCC_AHB2ENR;
	uint32_t RCC_AHB3ENR;
	uint32_t RCC_APB1ENR1;
	uint32_t RCC_APB1ENR2;
	uint32_t RCC_APB2ENR;
	uint32_t RCC_AHB1SMENR;
	uint32_t RCC_AHB2SMENR;
	uint32_t RCC_AHB3SMENR;
	uint32_t RCC_APB1SM;
	uint32_t RCC_APB1SMENR2;
	uint32_t RCC_APB2SMENR;
	uint32_t RCC_CCIPR;
	uint32_t RCC_BDCR;
	uint32_t RCC_CSR;
	uint32_t RCC_CRRCR;
	uint32_t RCC_CCIPR2;
	uint32_t RCC_DLYCFGR;
}RCC_Reg_Def_t;

/*
 * Peripheral register definition structure for SPI
 * */
typedef struct{
	_vo uint16_t CR1;
	_vo uint16_t CR2;
	_vo uint16_t SR;
	_vo uint16_t DR;
	_vo uint16_t CRCPR;
	_vo uint16_t RXCRCR;
	_vo uint16_t TXCRCR;
}SPI_Reg_Def_t;

/*
 * SPI peripheral definitions
 * */
#define SPI1 	(SPI_Reg_Def_t*)SPI1_BASEADDR
#define SPI2	(SPI_Reg_Def_t*)SPI2_BASEADDR
#define SPI3	(SPI_Reg_Def_t*)SPI3_BASEADDR

/*
 * Bit position definitions of SPI peripheral
 * */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCN_EXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15
/*
 * Bit position definitions SPI_SR
 * */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
/*
 * Bit position definitions SPI_CR2
 * */
#define SPI_CR2_DS			8
#define SPI_CR2_TXEIE 		11
#define SPI_CR2_RXEIE		6
#define SPI_CR2_ERRIE		5
/*
 * Peripheral register definition structure for EXTI
 * */
typedef struct{
	_vo uint32_t IMR; 				//EXTI_IMR1 P481rm0432 offset 0x00
	_vo uint32_t EMR; 				//EXTI_EMR1 P481rm0432		  0x04
	_vo uint32_t RTSR; 			 	//EXTI_RTSR1 P481rm0432		  0x08
	_vo uint32_t FTSR; 				//EXTI_FTSR1 P481rm0432		  0x0C
	_vo uint32_t SWIER; 			//EXTI_SWIER1 P481rm0432	  0x10
	_vo uint32_t PR; 				//EXTI_PR1 P481rm0432		  0x14

}EXTI_Reg_Def_t;

/*
 * Peripheral register definition structure for SYSCFG
 * */
typedef struct{
	_vo uint32_t MEMRMP; 				//P364rm0432 offset 	  0x00
	_vo uint32_t CFGR1; 				//P364rm0432			  0x04
	_vo uint32_t EXTICR[4]; 			//P364rm0432		  	  0x08-0x14
	_vo uint32_t SCSR;					//						  0x18
	_vo uint32_t CFGR2;					//						  0x1C
	_vo uint32_t SWPR;  				// 						  0x20
	_vo uint32_t SKR;
	_vo uint32_t SWPR2;

}SYSCFG_Reg_Def_t;
/*
 * Peripherals definition for RCC
 * */

#define RCC							((RCC_Reg_Def_t*)RCC_BASEADDR)
#define EXTI						((EXTI_Reg_Def_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_Reg_Def_t*)SYSCFG_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<0)) 		//P288rm0432
#define GPIOB_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<1))		//GPIOxEN
#define GPIOC_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<7))
#define GPIOI_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1<<8))

/*
 * Clock Disable Macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<0)) 		//P288rm0432
#define GPIOB_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<1))		//GPIOxEN
#define GPIOC_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1<<8))


/*
 * Clock Enable Macros for I2Cx peripherals
 * */
#define I2C1_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<21))		//P293rm0432
#define I2C2_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<22))		//I2C1,2,3,4EN
#define I2C3_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<23))
#define I2C4_PCLK_EN()				(RCC->RCC_APB1ENR2 |= (1<<1))

/*
 * Clock Disable Macros for I2Cx peripherals
 * */
#define I2C1_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<21))		//P293rm0432
#define I2C2_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<22))		//I2C1,2,3,4EN
#define I2C3_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<23))
#define I2C4_PCLK_DI()				(RCC->RCC_APB1ENR2 &= ~(1<<1))
/*
 * Clock Enable Macros for SPIx peripherals
 * */
#define SPI1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1<<12))		//P295rm0432
#define SPI2_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<14))		//SPIxEN
#define SPI3_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<15))
/*
 * Clock Disable Macros for SPIx peripherals
 * */
#define SPI1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1<<12))		//P295rm0432
#define SPI2_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<14))		//SPIxEN
#define SPI3_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<15))
/*
 * Clock Enable Macros for USARTx peripherals
 * */
#define USART1_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<14))
#define USART2_PCLK_EN()			(RCC->RCC_APB1ENR1 |= (1<<17))
#define USART3_PCLK_EN()			(RCC->RCC_APB1ENR1 |= (1<<18))
/*
 * Clock Disable Macros for USARTx peripherals
 * */
#define USART1_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()			(RCC->RCC_APB1ENR1 &= ~(1<<17))
#define USART3_PCLK_DI()			(RCC->RCC_APB1ENR1 &= ~(1<<18))
/*
 * Clock Enable Macros for UARTx peripherals
 * */
#define UART4_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<19))
#define UART5_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1<<20))
/*
 * Clock Disable Macros for UARTx peripherals
 * */
#define UART4_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<19))
#define UART5_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1<<20))
/*
 * Clock Enable Macros for SYSCFG peripherals
 * */
#define SYSCFG_PCLK_EN()			(RCC->RCC_APB2ENR |= (1<<0))
/*
 * Clock Disable Macros for SYSCFG peripherals
 * */
#define SYSCFG_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1<<0))
/*
 *
 * */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0:\
									 (x == GPIOB)?1:\
									 (x == GPIOC)?2:\
									 (x == GPIOD)?3:\
									 (x == GPIOE)?4:\
									 (x == GPIOF)?5:\
									 (x == GPIOG)?6:\
									 (x == GPIOH)?7:\
								     (x == GPIOI)?8:0)

/*
 * IRQ (Interrupt Request) Numbers of STM32L432xx MCU
 * */
#define IRQ_NO_EXTI0				6					//Search in P468rm0432
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

/*
 * Macros to reset GPIOx peripherals
 * */
#define GPIOA_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<0));(RCC->RCC_AHB2RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<1));(RCC->RCC_AHB2RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<2));(RCC->RCC_AHB2RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<3));(RCC->RCC_AHB2RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<4));(RCC->RCC_AHB2RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<5));(RCC->RCC_AHB2RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<6));(RCC->RCC_AHB2RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<7));(RCC->RCC_AHB2RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()			do{(RCC->RCC_AHB2RSTR |= (1<<8));(RCC->RCC_AHB2RSTR &= ~(1<<8));}while(0)

/*
 * Some generic macros
 * */

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET 	   RESET
#define FLAG_SET	   SET

#endif /* INC_STM32L432XX_H_ */
