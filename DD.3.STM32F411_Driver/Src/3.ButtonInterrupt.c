/**
 ******************************************************************************
 * @file           : 3.ButtonInterrupt.c
 * @author         : Sunil Kumar Yadav
 * @Date           : 29 May 2022
 * @brief          : Program to toggle LED on discovery board when external button is pressed.
 * 					 Interrupt will be triggered during falling edge of button press.
 *
 *  Note: PD12 is on-board LED which I'm using in this exercise and PD5 for external button
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>               // memset()
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


	// reset local variables to default initialize to 0
	memset(&ledGPIO,0, sizeof(ledGPIO) );
	memset(&buttonGPIO,0, sizeof(buttonGPIO) );

	// Configuration for LED connected to GPIO PD12
	ledGPIO.pGPIOx = GPIOD;
	ledGPIO.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;          // refer schematic diagram from stm32 Discovery user manual
	ledGPIO.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUT;          // Crucial to set mode to output mode
	ledGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledGPIO.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	ledGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);                     // enable peripheral clock
	GPIO_Init(&ledGPIO); 									  // initialize the gpio for led pin

    //-------------------------------------------------------------------------------------------------------------------------------------
	// Configuration for push button connected to GPIO PD5  with GPIO mode interrupt falling edge
	buttonGPIO.pGPIOx = GPIOD;
	buttonGPIO.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_5;          // refer schematic diagram from stm32 Discovery user manual. PA0
	buttonGPIO.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;       // Input mode is set to interrupt falling edge
	//buttonGPIO.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   //  Not applicable
	buttonGPIO.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	buttonGPIO.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;    // external pull up configuration


	GPIO_PeriClockControl(GPIOD, ENABLE);                           // enable peripheral clock
	GPIO_Init(&buttonGPIO);

	// configure IRQ
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI_15);

	for(;;);
}

/* Implementation of IRQ handler*/
void EXTI9_5_IRQHandler(void) {
	delaySW();                               // To avoid de-bouncing of button, add delay for ~100ms.
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);

}
