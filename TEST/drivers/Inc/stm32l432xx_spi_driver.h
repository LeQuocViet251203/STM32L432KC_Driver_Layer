/*
 * stm32l432xx_spi_driver.h
 *
 *  Created on: Mar 18, 2025
 *      Author: ADMIN
 */

#ifndef INC_STM32L432XX_SPI_DRIVER_H_
#define INC_STM32L432XX_SPI_DRIVER_H_
#include "Stm32l432xx.h"
/*
 * Configuration structure for SPIx peripheral
 * */
typedef struct{
	uint8_t SPI_DeviceMode;					//CR1 control register bit
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;					//Bit 5:3
	uint8_t SPI_DFF;						//Bit 11
	uint8_t SPI_CPOL;						//Bit 1
	uint8_t SPI_CPHA;						//Bit 0
	uint8_t SPI_SSM;						//Bit 9
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 * */
typedef struct{
	SPI_Reg_Def_t 		*pSPIx;
	SPI_Config_t 		SPIConfig;
	uint8_t 			*pTxBuffer; 		//store the app. Tx buffer address
	uint8_t 			*pRxBuffer; 		//Store teh app. Rx buffer address
	uint8_t 			TxLen;				// store the Tx Len
	uint8_t 			RxLen;				//Store the Rx Len
	uint8_t				TxState;			//Store the Tx state
	uint8_t				RxState;			//Store the Rx State
}SPI_Handle_t;
/*
 * @DeviceMode
 * */
#define SPI_DEVICE_MODE_MASTER 			1	//P1897rm0432
#define SPI_DEVICE_MODE_SLAVE			0
/*
 * @BusConfig
 * */
#define SPI_BUS_CONFIG_FD				1	//P1897rm0432
#define SPI_BUS_CONFIG_HD				2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3
/*
 * @SclkSpeed
 * */
#define SPI_SCLK_SPEED_DIV2				0	//P1897rm0432
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * Possible SPI Application events
 * */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4
/*
 * @CPHA
 * */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0
/*
 * @CPOL
 * */
#define SPI_CPOL_HIGH 					1
#define SPI_CPOL_LOW 					0
/*
 * @DFF
 * */
#define SPI_DFF_8BITS 					0
#define SPI_DFF_16BITS 					1
/*
 * @SSM
 * */
#define SPI_SSM_HW						1	//Hardware
#define SPI_SSM_SW						0	//Software
/*
 * SPI related status flags definitions
 * */
#define SPI_TXE_FLAG					(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1<<SPI_SR_BSY)
/*
 * APIs supported by this driver
 * The functions prototype - APIs
 * */
/*
 * Peripheral Clock setup
 * */
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi);
/*
 * Init and De-init
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle); //Function to initialize port and pin for intended purposes
void SPI_DeInit(SPI_Reg_Def_t *pSPIx); //Reset to the reset state - RCC_AHB2RSTR - RCC AHB2 peripheral reset register
uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName);
/*
 * Data send and Receive
 * */
void SPI_SendData(SPI_Reg_Def_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);
void SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR handling
 * */
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);//,uint8_t IRQPriority deleted to separate to a diff func
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * Other peripheral control APIs
 * */
void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnOrDi);
void SPI_CLearOVRFlag(SPI_Reg_Def_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * Application callback
 * */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
#endif /* INC_STM32L432XX_SPI_DRIVER_H_ */

