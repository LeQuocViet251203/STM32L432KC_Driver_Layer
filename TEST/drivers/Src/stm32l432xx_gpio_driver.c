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
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Configure the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Configure the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//Configure both FTSR and RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);
		//Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
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
/*
 * GPIO Reset Register
 * */
/*
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function reset all the registers of the GPIO port
 *
 * @param[in]	- Base address of the GPIO peripheral
 *
 *
 * @return 		- none
 *
 * @note		- In order to reset the register of the GPIO pins we need
 * 				to refer to the respective RCC register in the RCC block
 * 				RCC_AHB2RSTR P279rm0432
 * */
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx){
		if(pGPIOx == GPIOA) GPIOA_REG_RESET();
		else if(pGPIOx == GPIOB) GPIOB_REG_RESET();
		else if(pGPIOx == GPIOC) GPIOC_REG_RESET();
		else if(pGPIOx == GPIOD) GPIOD_REG_RESET();
		else if(pGPIOx == GPIOE) GPIOE_REG_RESET();
		else if(pGPIOx == GPIOF) GPIOF_REG_RESET();
		else if(pGPIOx == GPIOG) GPIOG_REG_RESET();
		else if(pGPIOx == GPIOH) GPIOH_REG_RESET();
		else if(pGPIOx == GPIOI) GPIOI_REG_RESET();
}
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value =(uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001);
	return value;
}

/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx){
	uint16_t value;
	value =(uint16_t)pGPIOx->IDR;
	return value;
}

/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		//Write 1 to the output data register at the bit field corressponding to the pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}else if(Value == GPIO_PIN_RESET){
		// Write 0
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR |= Value;
}
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi){
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
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	// Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section) + (8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+(iprx*4)) |= (IRQPriority << (8*shift_amount));
}
/*
 * GPIO Read from input pin
 * */
/*
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function read the data from the pin that is registered
 *
 * @param[in]	- Base address of the GPIO peripheral
 * @param[in]	- The pin number
 *
 * @return 		- 0 or 1
 *
 * @note		- GPIOx_IDR P344rm0432
 * */
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1<<PinNumber)){
		// Clear
		EXTI->PR |= (1<<PinNumber);
	}
}
