/**
 ******************************************************************************
 * @file           : 2.ledWithButton.c
 * @author         : Sunil Kumar Yadav
 * @Date           : 29 May 2022
 * @brief          : Program to toggle LED on discovery board when button is pressed.
 *
 *  Note: In Logic analyzer connect ground pin with ground on the board and PA8
 *  to one of the channel for measurement.
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f411.h"
#include "stm32f411_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif



#define TRUE              1
#define BUTTON_PRESSED    TRUE
// User Push button: User and Wake-Up button connected to the I/O PA0 of the STM32F411VE. External pull down resistor available.

// software delay for certain fraction of second
void delaySW(void){

	for(uint32_t i=0; i< 500000/2; ++i);
}

int main(void)
{

	GPIO_Handle_t ledGPIO, buttonGPIO;

	// Configuration for LED connected to GPIO PD12
	ledGPIO.pGPIOx = GPIOD;
	ledGPIO.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;          // refer schematic diagram from stm32 Discovery user manual
	ledGPIO.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUT;          // Crucial to set mode to output mode
	ledGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledGPIO.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ledGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);                     // enable peripheral clock
	GPIO_Init(&ledGPIO); 									  // initialize the gpio for led pin

    //-------------------------------------------------------------------------------------------------------------------------------------
	// Configuration for push button connected to GPIO PA0
	buttonGPIO.pGPIOx = GPIOA;
	buttonGPIO.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_0;          // refer schematic diagram from stm32 Discovery user manual. PA0
	buttonGPIO.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_IN;          // Crucial to set mode to Input mode for push button
	//buttonGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   //  Not applicable
	buttonGPIO.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	buttonGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;   // Not required as already pull down register available


	GPIO_PeriClockControl(GPIOA, ENABLE);                     // enable peripheral clock
	GPIO_Init(&buttonGPIO);


	while(1){

		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) == BUTTON_PRESSED) {
			delaySW();                                    // Delay to avoid denouncing side effect of button
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		}
	}

	for(;;);
}
