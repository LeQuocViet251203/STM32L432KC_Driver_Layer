/*
 * stm32l432xx_gpio_driver.h
 *
 *  Created on: Mar 16, 2025
 *      Author: ADMIN
 */

#ifndef INC_STM32L432XX_GPIO_DRIVER_H_
#define INC_STM32L432XX_GPIO_DRIVER_H_

#include "Stm32l432xx.h"
#include <stdint.h>

/*
 * GPIO Configuration structure for a GPIO pin
 * */
typedef struct{
	uint8_t GPIO_PinNumber;			//Possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;			//Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	//Possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			//Possible values from @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct{
	// pointer to hold the base addr of the GPIO peripherals
	GPIO_Reg_Def_t *pGPIOx; // This holds the base addr of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 * */
#define GPIO_MODE_IN 0		//Input mode P342rm0432	//Can also configure the GPIO pin to deliver an interrupt to the processor to detect fallin or risin edge
#define GPIO_MODE_OUT 1		//General purpose output mode
#define GPIO_MODE_ALT 2		//Alternate function mode
#define GPIO_MODE_ANALOG 3 	//Analog mode(reset state)		   // Non-interrupt modes
#define GPIO_MODE_IT_FT 4	//Falling edge detected on the pin // Interrupt modes
#define GPIO_MODE_IT_RT	5	//Rising edge detected on the pin
#define GPIO_MODE_IT_RFT 6
/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 * */
#define GPIO_OP_TYPE_PP	0	//Output push-pull(reset state)
#define GPIO_OP_TYPE_OD	1	//Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed
 * */
#define GPIO_SPEED_LOW		0	//Low speed
#define GPIO_SPEED_MEDIUM	1	//Medium speed
#define GPIO_SPEED_HIGH		2	//High speed
#define GPIO_SPEED_VEHIGH	3	//Very high speed

/*
 * @GPIO_PIN_PUPD
 * GPIO pin possible pull-up/pull-down
 * */
#define GPIO_NO_PUPD	0	//No pull-up,pull-down
#define GPIO_PIN_PU			1	//Pull-up
#define GPIO_PIN_PD			2	//Pull-down

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin numbers from 0 - 15
 * */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*
 * APIs supported by this driver
 * The functions prototype - APIs
 * */
/*
 * Peripheral Clock setup
 * */
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx,uint8_t EnorDi);
/*
 * Init and De-init
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle); //Function to initialize port and pin for intended purposes
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx); //Reset to the reset state - RCC_AHB2RSTR - RCC AHB2 peripheral reset register

/*
 * Data read and write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber, uint8_t Value);// Value for GPIO_PIN_SET and GPIO_PIN_RESET
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx,uint8_t PinNumber);
/*
 * IRQ Configuration and ISR handling
 * */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
#endif /* INC_STM32L432XX_GPIO_DRIVER_H_ */
