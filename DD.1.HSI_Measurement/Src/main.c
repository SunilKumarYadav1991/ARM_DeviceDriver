/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sunil Kumar Yadav
 * @Date           : 14Jan2022
 * @brief          : Program to understand how to configure the clock on STM32
 *  and use logic analyzer to measure the HSI clock frequency using MCOx pin
 *  (Microcontroller Clock Output). HSI in this board is 16MHZ.
 *  By default HSI is used
 *
 *  Note: In Logic analyzer connect ground pin with ground on the board and PA8
 *  to one of the channel for measurement.
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


#define RCC_BASE_ADDR             0x40023800UL                             /* RCC base register address*/
#define RCC_CFGR_REG_OFFSET       0x8UL                                    /* RCC clock configuration register offset*/
#define RCC_CFGR_REG_ADDR         (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)    /* RCC clock configuration register address*/
#define GPIOA_BASE_ADDR           0x40020000UL                             /* Base address of GPIO A*/

int main(void)
{

    // 1. Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
	volatile uint32_t *pRccCfgrReg = (volatile uint32_t*)RCC_CFGR_REG_ADDR;

	*pRccCfgrReg &= ~(0x3 << 21);                    /* MCO1 clear 21 and 22 bit positions. 00: HSI clock selected, 01: LSE oscillator selected
	                                                  10: HSE oscillator clock selected, 01: LSE oscillator selected*/

	*pRccCfgrReg  |= (1 << 25);                     /* configure MCO1 prescaler: bit 25-26 = 1 --> MCO = System clock(HSI 16MHz)/4 == 4MHZ*/
	*pRccCfgrReg  |= (1 << 26);


	// 2. Configure PA8 pin to AF0 mode (alternate mode) to behave as MCO1 signal. Refer datasheet for more details
	// 2.1 Before accessing any peripheral/GPIO, we need to enable respective bus on which its connected. GPIO connected to AHB1bus
	volatile uint32_t *pRccAbh1Enr = (volatile uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRccAbh1Enr  |= (1 << 0);                     /* enable GPIO A peripheral clock*/

	// 2.2 Configure the mode of GPIOA pin 8 as alternate function mode
	volatile uint32_t *pGPIOAModeReg = (volatile uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*pGPIOAModeReg &= ~(0x3 << 16);              /*clear*/
	*pGPIOAModeReg |= (0x2 << 16);               /*set bit 16:17 GPIO A8. 10: Alternate function mode, 11: Analog mode etc*/

	// 2.3 Configure the alternation function register to set the mode 0 for PA8
	volatile uint32_t *pGPIOAAltFunctionHighReg = (volatile uint32_t*)(GPIOA_BASE_ADDR + 0x24);
	*pGPIOAAltFunctionHighReg &= ~(0xf << 0);         /*0000:AF0 .. 1111:AF15*/

	for(;;);
}
