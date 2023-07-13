/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sunil Kumar Yadav
 * @Date           : 29 May 2022
 * @brief          : Program to toggle LED on discovery board.
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


// sofware delay for certain fraction of second
void delaySW(void){

	for(uint32_t i=0; i< 500000; ++i);
}

int main(void)
{

	GPIO_Handle_t ledGPIO;

	ledGPIO.pGPIOx = GPIOD;
	ledGPIO.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;          // refer schematic diagram from stm32 Discovery user manual
	ledGPIO.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUT;          // Crucial to set mode to output mode
	ledGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledGPIO.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ledGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// use below config for open drain output configuration.
	// NOTE:  In order to use open drain mode use external VCC with lower resister value on required pin. i.e. VCC-----^^^-----PIN
	//ledGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	//ledGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);                     // enable peripheral clock

	GPIO_Init(&ledGPIO); 									  // initialize the gpio for led pin


	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delaySW();
	}

	for(;;);
}
