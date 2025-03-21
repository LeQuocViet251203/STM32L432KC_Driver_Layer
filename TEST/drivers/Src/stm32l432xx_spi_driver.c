/*
 * stm32l432xx_spi_driver.c
 *
 *  Created on: Mar 18, 2025
 *      Author: ADMIN
 */

#include <stdint.h>
#include "stm32l432xx_spi_driver.h"
#include "Stm32l432xx.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);
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

void SPI_DeInit(SPI_Reg_Def_t *pSPIx){
	if(pSPIx == SPI1);
//			else if(pGPIOx == GPIOB) GPIOB_REG_RESET();
//			else if(pGPIOx == GPIOC) GPIOC_REG_RESET();
//			else if(pGPIOx == GPIOD) GPIOD_REG_RESET();
//			else if(pGPIOx == GPIOE) GPIOE_REG_RESET();
//			else if(pGPIOx == GPIOF) GPIOF_REG_RESET();
//			else if(pGPIOx == GPIOG) GPIOG_REG_RESET();
//			else if(pGPIOx == GPIOH) GPIOH_REG_RESET();
//			else if(pGPIOx == GPIOI) GPIOI_REG_RESET();
}
uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
	return FLAG_SET;
	}
	return FLAG_RESET;
}
//Blocking API
void SPI_SendData(SPI_Reg_Def_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		// wait until TXE is set
		//while(!(pSPIx->SR &(1<<1))); // Implement a function
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)== FLAG_RESET);
		// Check the DFF bit in CR1
		if(pSPIx->CR1&(1<<SPI_CR2_DS)){
			//16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		// wait until TXE is set
		//while(!(pSPIx->SR &(1<<1))); // Implement a function
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)== FLAG_RESET);
		// Check the DFF bit in CR1
		if(pSPIx->CR1&(1<<SPI_CR2_DS)){
			//16 bit DFF
			// load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}
void SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
	// Save the Tx buffer address and Len information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	// Mark the SPI state as busy in transmission so that no other
	//code can take over same SPI peripheral until transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in
	// SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	// Data transmission will be handled by the ISR code
	}
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
	// Save the Tx buffer address and Len information in some global variables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;
	// Mark the SPI state as busy in transmission so that no other
	//code can take over same SPI peripheral until transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;
	// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in
	// SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXEIE);
	// Data transmission will be handled by the ISR code
	}
	return state;
}
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			//Program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ISER1 register
			*NVIC_ISER1 |= (1<<IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber <96){
			//Program ISER2 register
			*NVIC_ISER3 |= (1<<IRQNumber % 64);
		}
	}else{
		if(IRQNumber <= 31){
			//Program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			//Program ICER1 register
			*NVIC_ICER1 |= (1<<IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber <96){
			//Program ICER2 register
			*NVIC_ICER3 |= (1<<IRQNumber % 64);
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	// Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section) + (8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+(iprx*4)) |= (IRQPriority << (8*shift_amount));
}
void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1,temp2;
	//First lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXEIE);
	if(temp1 && temp2){
		//handle RXE
		spi_rxne_interrupt_handle(pHandle);
	}
	// Check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
		//handle RXE
		spi_ovr_interrupt_handle(pHandle);
	}
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_DS)){
				//16 bit DFF
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}else{
				//8 bit DFF
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if(!pSPIHandle->TxLen){
		//TxLen is zero, so close the spi transmission and inform the application
		//that TX is over
		//Prevent interrupts from setting up of TXE flag
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL ;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_DS)){
				//16 bit DFF
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
				pSPIHandle->RxLen--;
				pSPIHandle->RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}else{
				//8 bit DFF
				pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
			}
	if(!pSPIHandle->RxLen){
		//TxLen is zero, so close the spi transmission and inform the application
		//that TX is over
		//Prevent interrupts from setting up of TXE flag
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXEIE);
		pSPIHandle->pRxBuffer = NULL ;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//Clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//Inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
}
void SPI_CLearOVRFlag(SPI_Reg_Def_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
			pSPIHandle->pTxBuffer = NULL ;
			pSPIHandle->TxLen = 0;
			pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXEIE);
			pSPIHandle->pRxBuffer = NULL ;
			pSPIHandle->RxLen = 0;
			pSPIHandle->RxState = SPI_READY;
}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	//This is a weak implementation. The application may override this function

}
