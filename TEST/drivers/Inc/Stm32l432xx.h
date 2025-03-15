/*
 * Stm32l432xx.h
 *
 *  Created on: Mar 15, 2025
 *      Author: ADMIN
 */

#ifndef INC_STM32L432XX_H_
#define INC_STM32L432XX_H_

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
 *
 * */


#endif /* INC_STM32L432XX_H_ */
