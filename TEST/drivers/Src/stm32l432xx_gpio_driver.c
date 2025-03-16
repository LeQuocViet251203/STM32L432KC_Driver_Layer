/*
 * stm32l432xx_gpio_driver.c
 *
 *  Created on: Mar 16, 2025
 *      Author: ADMIN
 */


#include "stm32l432xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 * */
/*
 * @fn			- GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Enable or Disable macros
 *
 *
 * @return 		- none
 *
 * @note		- none
 * */
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH) GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI) GPIOI_PCLK_EN();
	}
}

/*
 * GPIO Initialization Function
 * */
/*
 * @fn			- GPIO_Init
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- Enable or Disable macros
 *
 *
 * @return 		- none
 *
 * @note		- none
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0 ;
	// Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		/*
		 * pGPIOHandle is the struct for the address of the GPIOx base addr
		 * and the configuration for the GPIOx
		 * This if statement means if the PinMode(@GPIO_PIN_MODES) require
		 * interrupt mode which is the larger than 3(GPIO_MODE_ANALOG) then
		 * the code block here is execute
		 * */
		//The non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		/*
		 * Looking at the GPIOx_MODER register at P342rm0432
		 * if the pinmode is 5 then it will move 10 bits to the left and
		 * stop at MODE5
		 * */
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bit field to start working
		pGPIOHandle->pGPIOx->MODER |= temp; // We dont use the = operator as we dont have the authority to change the whole registered bits field but to modify the place that we want only
		/*
		 * Configure it into the baseaddress of the GPIOx pin
		 * */
	}else{
		//The interrupt mode
	}
	temp = 0;
	// Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	// Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	// Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	// Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT ){
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x15 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));
	}
}
