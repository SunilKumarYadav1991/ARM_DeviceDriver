/*
 * stm32f411_gpio_driver.c
 *
 *  Created on: 23-May-2023
 *      Author: Sunil Kumar Yadav
 */


#include "stm32f411_gpio_driver.h"

/* GPIO Peripheral clock setup*/
/**
 * GPIO_PeriClockControl() function enables or disables peripheral clock for given GPIO port
 * @param pGPIOx            base address of GPIO peripheral
 * @param enableOrDisable   boolean 1 to enable/set and 0 to disable/reset
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enableOrDisable){

	if(enableOrDisable== ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else {
			// wrong port address
		}

	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else {
			// wrong port address
		}

	}

}


/* GPIO Init and De-Init*/
/**
 * GPIO_Init() function initializes the GPIO port with respective mode
 * @param pGPIOHandle      GPIO handle with base address to GPIO with respective mode
 *
 * TODO: Interrupt mode implementation pending
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;
	uint8_t pinNumber = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber;             // pin number of port for which initialization is required

	//1. Configure the mode of gpio pin: focusing on non interrupt modes as of now
	if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode <= GPIO_MODE_AALOG) {           // non interrupt mode.

		temp =  pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2*pinNumber);      // Configure respective pin mode field.  2bit per pin hence multiply by 2

		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pinNumber);                     // Reset/clear existing value
		pGPIOHandle->pGPIOx->MODER |= temp;                                     // write MODER register value to respective GPIO register
		temp = 0;
	} else {

		// clear rising and falling trigger selection register before setting them.
		EXTI->FTSR &= ~(1 << pinNumber);
		EXTI->RTSR &= ~(1 << pinNumber);

		if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. configure the Falling trigger selection register (EXTI_FTSR)
			EXTI->FTSR |= (1 << pinNumber);

		} else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. configure the Rising trigger selection register (EXTI_RTSR)
			EXTI->RTSR |= (1 << pinNumber);

		} else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 1. configure the both Rising and Falling trigger selection register (EXTI_RTSR, EXTI_FTSR)
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);

		}

		// 2. Configure  the GPIO port selection in sys config exti config register SYSCFG_EXTICR

		SYSCFG_PCLK_EN();                                                        // Enable SYSCFG peripheral clock

		uint8_t idxToEXTICR = pinNumber/4;                                       // EXTICR contains 4 registers i.e. SYSCFG_RegDef_t.EXTICR[4] with each upper 16bit reserved. 4*4 bit for each EXTIx
		uint8_t idEXTIxBit = pinNumber%4;                                        // EXTIx bit position to be updated for GPIOx mapping into EXTI. Refer 7.2. SYSCFG external interrupt configuration register@reference manual

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);           // get port code to be written into EXTICR[x] register to enable configuration of given GPIO pin to EXTIx

		SYSCFG->EXTICR[idxToEXTICR] = portCode << (idEXTIxBit*4);

		// 3. Enable the EXT interrupt delivery using Interrupt mask register
		EXTI->IMR |= (1 << pinNumber);
	}


	//2. Configure the speed
	temp =  pGPIOHandle->GPIO_pinConfig.GPIO_PinSpeed << (2*pinNumber);          // Configure respective pin mode field. 2bit per pin hence multiply by 2

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pinNumber);                        // Reset/clear existing value
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;                                        // write SPEEDR register value to respective GPIO register
	temp = 0;


	//3. Configure the pull up/down settings
	temp =  pGPIOHandle->GPIO_pinConfig.GPIO_PinPuPdControl << (2*pinNumber);    // Configure respective pin mode field. 2bit per pin hence multiply by 2

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pinNumber);                          // Reset/clear existing value
	pGPIOHandle->pGPIOx->PUPDR |= temp;                                          // write PUPDR register value to respective GPIO register
	temp = 0;


	//4. Configure the output type
	temp =  pGPIOHandle->GPIO_pinConfig.GPIO_PinOPType << (pinNumber);           // Configure respective pin mode field. 1bit per pin hence multiply by 1

	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber);                          // Reset/clear existing value
	pGPIOHandle->pGPIOx->OTYPER |= temp;                                         // write OTYPER register value to respective GPIO register
	temp = 0;


	//5. Configure the alternate functionality (if required)
	if(pGPIOHandle->GPIO_pinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN) {      // AFRL and AFRH contains Alt function mode in 4 bit per pin.

		uint8_t offset = pinNumber % 8;                                          // offset factor within AFRL/AFRH register
		temp =  pGPIOHandle->GPIO_pinConfig.GPIO_PinAltFunMode << (4*offset);    // Configure respective pin mode field.  4bit per pin hence multiply by 4

		if(pinNumber <= 7) {
			pGPIOHandle->pGPIOx->AFRL &= ~(0x0F << pinNumber);                   // Reset/clear existing value
			pGPIOHandle->pGPIOx->AFRL |= temp;                                   // write AFRL register value to respective GPIO register
		} else {
			pGPIOHandle->pGPIOx->AFRH &= ~(0x0F << pinNumber);                   // Reset/clear existing value
			pGPIOHandle->pGPIOx->AFRH |= temp;                                   // write AFRH register value to respective GPIO register
		}
	}


}


/**
 * GPIO_DeInint() function de-initializes the GPIO port
 * @param pGPIOx        base address of GPIO peripheral
 * Note: DeInint function resets RCC AHB1STR register to 1 to reset IO and then 0 to not reset IO port
 */
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOA_REG_RESET();
	} else {
		// wrong port address
	}

}


/* GPIO Data read and write*/
/**
 * GPIO_ReadFromInputPin() function reads data from GPIO input pin
 * @param pGPIOx        base address of GPIOx peripheral
 * @param pinNumer      pin number of GPIOx port
 * @return              value 1/0 at pin of port
 *
 * Note: Refer GPIO port input data register (GPIOx_IDR) for more details
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){

	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR >>pinNumber) & 0x01;     // GPIOx_IDR bit 31-16 are reserved. Shifting value of bit position by pin number to extract value
	return value;
}


/**
 * GPIO_ReadFromInputPort() function reads data from GPIO input port
 * @param pGPIOx        base address of GPIOx peripheral
 * @return              content of 16 bit port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR) ;                   // GPIOx_IDR bit 31-16 are reserved. Type casting with uint16_t will capture bit 0-15
	return value;
}


/**
 * GPIO_WriteToInputPin() function write data from GPIO output pin
 * @param pGPIOx        base address of GPIOx peripheral
 * @param pinNumer      pin number of GPIOx port
 * @param value         Value to be written to GPIOx pin (0|1)
 */
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){

	if(value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1<<pinNumber);                 // write 1 to bit position of corresponding pin number.
	} else {
		pGPIOx->ODR &= ~(1<<pinNumber);                // write 0 to bit position of corresponding pin number.
	}
}


/**
 * GPIO_WriteToInputPort() function reads data from GPIO output port
 * @param pGPIOx        base address of GPIOx peripheral
 * @param value         Value to be written to GPIOx port, 16bit (0|1)
 *
 * Note: Bit 31-16 are reserved. Refer GPIO port output data register (GPIOx_ODR) @ Reference manual
 */
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}


/**
 * GPIO_ToggleOutputPin() function toggles output pin value
 * @param pGPIOx        base address of GPIOx peripheral
 * @param pinNumer      pin number of GPIOx port
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1<<pinNumber);                    // xor the pin value to toggle the output pin
}


/* GPIO IRQ configuration and ISR handling */
/**
 * GPIO_IRQInterruptConfig() configures the IRQ for GPIO
 * @param IRQNumber         IRQ number which will be configured for interrupt handling
 * @param enableOrDisable   enable or disable the IRQ associated with GPIO
 * Note: Refer generic Cortex M4 programming manual: 4.3 NVIC (4.3.2 section Interrupt set-enable register x (NVIC_ISERx) and 4.3. Interrupt clear-enable register x (NVIC_ICERx))
 *       Total IRQ no. implemented in STM32F411 is 92
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enableOrDisable){

	if(enableOrDisable == ENABLE) {
        if(IRQNumber<=31) {
        	// program ISER0
        	*NVIC_ISER0 |= (1 << IRQNumber);
        } else if(IRQNumber > 31 && IRQNumber < 64) {
        	// program ISER1
        	*NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if(IRQNumber >= 64 && IRQNumber < 96) {
        	// program ISER2
        	*NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }

	} else {
		 if(IRQNumber<=31) {
		     // program ICER0
			 *NVIC_ICER0 |= (1 << IRQNumber);
		 } else if(IRQNumber > 31 && IRQNumber < 64) {
		     // program ICER1
			 *NVIC_ICER1 |= (1 << (IRQNumber % 32));
		 } else if(IRQNumber >= 64 && IRQNumber < 96) {
		    // program ICER2
			 *NVIC_ICER2 |= (1 << (IRQNumber % 32));
		 }
	}
}


/**
 * GPIO_IRQPriorityConfig() configures the IRQ priority for GPIO interrupt
 * @param IRQNumber         IRQ number which will be configured for interrupt handling
 * @param IRQPriority       IRQ priority (uint32_t as we will perform shift operation
 * Note: Refer generic Cortex M4 programming manual: 4.3 NVIC (4.3.7 Interrupt priority register x (NVIC_IPRx) @ Generic ref manual (PM214))
 *       Each NVIT_IPRx register span from 0 to 59 (32 bit) with each register divided into 4 section (8 bit per IRQ Interrupt no.)
 *
 * Example: Setting priority for IRQ number 5 with priority 0xX0(lower 4 bits are not implemented)
 * 1. Find NVIC_IPRx register (0-59)
 *    iprx = IRQ number / 4 = 5/4 =1                     => NVIC_IPR1
 * 2. Calculate address of NVIC_IPR1
 *    NVIC_PRIORITY_BASE_ADDR + (iprx*4)                 // 32bit registers and hence next hex address.
 * 3. Calculate offest/section within NVIC_IPRx register
 *    iprxSection = IRQ number % 4 = 5%4 = 1            => NVIC_IPR1[8:16] bit 8-16 holds priority for IRQ5 inNVIC_IPR1 register
 * 4. Update the value to required register by left shifting iprxSection*8 + no. of bits implemented as per microcontroller
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// 1. Find IPR register
	uint8_t iprx = IRQNumber / 4;                                             // find NVIC_IPRx register based on IRQ no.
	uint8_t addressOffset = iprx;                                             // offset to be added into base address of NVIC_IPR reg to reach NVIC_IPRx
	uint8_t iprxSection = IRQNumber % 4;                                      // find section within NVIC_IPRx register based on IRQ no.

	volatile uint32_t* NVIC_priorityRegAddress = NVIC_PRIORITY_BASE_ADDR;

	// Calculate shift within section. Since bits[3:0] are not implemented, adjust the no. of times bits needs to shift i.e. extra 4 bit shift within section to avoid writing to bit[3:0]
	uint8_t shiftInSection = (8 * iprxSection) + (8 - NUMBER_OF_PRIORITY_BIT_IMPLEMENTED);

	// Assign interrupt priority value to NVIC_IPRx register address
	*(NVIC_priorityRegAddress + addressOffset) |=  (IRQPriority << shiftInSection);

}

/**
 * GPIO_IRQHandling() gets called to service the GPIO interrupt
 * @param pinNumer          pin number associated with the IRQ for GPIO port's pin
 * Note: This function will get called from EXTI ISR function e.g. void EXTI0_IRQHandler(void).
 */
void GPIO_IRQHandling(uint8_t pinNumber) {
	// Clear the EXTI Pend Register corresponding to the pin number: refer 10.3.6 Pending register (EXTI_PR)
    if(EXTI->PR & (1<<pinNumber)){
    	// Clear the pend register for detected event by setting it 1
    	EXTI->PR |= (1<<pinNumber);
    }

}
