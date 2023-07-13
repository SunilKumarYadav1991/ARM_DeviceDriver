/*
 * stm32f411_gpio_driver.h
 *
 *  Created on: 23-May-2023
 *      Author: Sunil Kumar Yadav
 *
 * GPIOx Peripheral have below configurable items:
 * 1. GPIO port name  2. GPIO pin number  3. GPIO mode  4. GPIO speed  5. GPIO output type  6. GPIO Pull up / pull down  7. GPIO Alt. function mode
 */

#ifndef INC_STM32F411_GPIO_DRIVER_H_
#define INC_STM32F411_GPIO_DRIVER_H_

#include "stm32f411.h"

/**
 * This is a Handle struct for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;                            /* Possible values from @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;                           /* Possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;                     /* pull-up or pull-down control. Possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;                          /* Output type. Possible values from @GPIO_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;                      /* Alternate function mode */
}GPIO_PinConfig_t;



typedef struct {
	GPIO_RegDef_t   *pGPIOx;                           /* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_pinConfig;                   /* This holds handle struct for GPIO to control the mode */

}GPIO_Handle_t;


/* @GPIO_PIN_MODE, GPIO pin possible mode. Refer 8.4.1 GPIO port mode register (GPIOx_MODER) @Reference manual*/
#define GPIO_MODE_IN              0
#define GPIO_MODE_OUT             1
#define GPIO_MODE_ALTFN           2
#define GPIO_MODE_AALOG           3
#define GPIO_MODE_IT_FT           4         /* Interrupt(IT) Falling Edge Trigger(FT)*/
#define GPIO_MODE_IT_RT           5         /* Interrupt(IT) Raising Edge Trigger(FT)*/
#define GPIO_MODE_IT_RFT          6         /* Interrupt(IT) Raising and Falling Edge Trigger(FT)*/


/*  @GPIO_OP_TYPE, GPIO pin possible output. Refer 8.4.2 GPIO port output type register (GPIOx_OTYPER) @Reference manual*/
#define GPIO_OP_TYPE_PP           0        /* Output Push-pull (reset state)*/
#define GPIO_OP_TYPE_OD           1        /* Output Open-drain*/


/*  @GPIO_PIN_SPEED, GPIO port possible speed. Refer 8.4.3 GPIO port output speed register (GPIOx_OSPEEDR) @Reference manual*/
#define GPIO_SPEED_LOW            0
#define GPIO_SPEED_MEDIUM         1
#define GPIO_SPEED_FAST           2
#define GPIO_SPEED_HIGH           3



/* @GPIO_PIN_PUPD, GPIO port possible pull-up/pull-down configuration. Refer 8.4.4 GPIO port pull-up/pull-down register (GPIOx_PUPDR) @Reference manual*/
#define GPIO_NO_PUPD              0         /* No pull-up/pull-down*/
#define GPIO_PIN_PU               1         /* pull-up*/
#define GPIO_PIN_PD               2         /* pull-down*/

/* @GPIO_PIN_NUMBER, GPIO pin number macros for 16 pins (0-15)*/
#define GPIO_PIN_0                0
#define GPIO_PIN_1                1
#define GPIO_PIN_2                2
#define GPIO_PIN_3                3
#define GPIO_PIN_4                4
#define GPIO_PIN_5                5
#define GPIO_PIN_6                6
#define GPIO_PIN_7                7
#define GPIO_PIN_8                8
#define GPIO_PIN_9                9
#define GPIO_PIN_10               10
#define GPIO_PIN_11               11
#define GPIO_PIN_12               12
#define GPIO_PIN_13               13
#define GPIO_PIN_14               14
#define GPIO_PIN_15               15



//------------------------------------------------------------------------------------------------------------------

/* GPIO Peripheral clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enableOrDisable);

/* GPIO Init and De-Init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx);              /* RCC AHB1/ABP peripheral reset register can be used to reset the GPIOx state to reset state */

/* GPIO Data read and write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumer);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);                                /* Port is 16 bit wide*/
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumer, uint8_t value);
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumer);


/* GPIO IRQ configuration and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enableOrDisable);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumer);

#endif /* INC_STM32F411_GPIO_DRIVER_H_ */
