/*
 * stm32l432xx_spi_driver.c
 *
 *  Created on: Mar 18, 2025
 *      Author: ADMIN
 */

#include <stdint.h>
#include "stm32l432xx_spi_driver.h"
#include "Stm32l432xx.h"

void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1) SPI1_PCLK_EN();
			else if(pSPIx == SPI2) SPI2_PCLK_EN();
			else if(pSPIx == SPI3) SPI3_PCLK_EN();
	}else{
			if(pSPIx == SPI1) SPI1_PCLK_DI();
			else if(pSPIx == SPI2) SPI2_PCLK_DI();
			else if(pSPIx == SPI3) SPI3_PCLK_DI();
	}
}
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//Configure teh SPI_CR1 register
	uint32_t tempreg = 0;
	//Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be clear
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1<<SPI_CR1_RX_ONLY);
	}
	// Configure the SPI serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	// Configure the DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL);
	// Configure teh CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	// Configure teh CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	pSPIHandle->pSPIx->CR1 |= tempreg; //BC freshly intializing the CR1 we can use = assignment
}
