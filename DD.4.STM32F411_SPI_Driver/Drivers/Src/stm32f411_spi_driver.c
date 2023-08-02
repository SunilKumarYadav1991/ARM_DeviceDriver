/*
 * stm32f411_spi_driver.c
 *
 *  Created on: 18-Jul-2023
 *      Author: Sunil Kumar Yadav
 *      Remark: SPI driver api implementation
 */

#include "stm32f411_spi_driver.h"


/**
 * SPI_PeriClockControl() function enables or disables peripheral clock for given SPI x peripheral
 * @param pSPIx             base address of SPIx peripheral
 * @param enableOrDisable   boolean 1 to enable/set and 0 to disable/reset
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enableOrDisable) {

	if(enableOrDisable== ENABLE) {
			if(pSPIx == SPI1) {
				SPI1_PCLK_EN();
			} else if (pSPIx == SPI2) {
				SPI2_PCLK_EN();
			} else if (pSPIx == SPI3) {
				SPI3_PCLK_EN();
			} else if (pSPIx == SPI4) {
				SPI4_PCLK_EN();
			} else if (pSPIx == SPI5) {
				SPI5_PCLK_EN();
			} else {
				// wrong port address
			}

		} else {
			if(pSPIx == SPI1) {
				SPI1_PCLK_DI();
			} else if (pSPIx == SPI2) {
				SPI2_PCLK_DI();
			} else if (pSPIx == SPI3) {
				SPI3_PCLK_DI();
			} else if (pSPIx == SPI4) {
				SPI4_PCLK_DI();
			} else if (pSPIx == SPI5) {
				SPI5_PCLK_DI();
			} else {
				// wrong port address
			}

		}

}

/* SPI Init and De-Init*/

/**
 * SPI_Init() function initializes the given SPI x peripheral
 * @param pSPIHandle            handle/structure to SPI peripheral containing configuration
 * TODO: Replace register's bit position with macro
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	// Configure  the CPI_CR1 register

	uint32_t tempReg = 0;            // temp variable holding register values


	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);                           // Enabling the clock to bus where SPI peripherals are attached.

	// 1. configure the device mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bug config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// BIDI MODE bit should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);

	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// BIDI MODE bit should be set
		tempReg |= (1 << SPI_CR1_BIDI_MODE);

	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// BIDI MODE bit should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);

		// RX ONLY bit should be set
		tempReg |= (1 << SPI_CR1_RX_ONLY);

	}

	// 3. Configure the SPI serial clock speed i.e. baud rate
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF i.e. Data Frame Format
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    //----------------------------------------------------------------------------------------

	// Assign collected settings/values to CR register
	pSPIHandle->pSPIx->CR1 = tempReg;

}


/**
 * SPI_DeInint() function de-initializes the given SPI peripheral
 * @param pSPIx        base address of SPIx peripheral
 * Note: DeInint function resets RCC APB1STR/APB2STR register to 1 to reset SPI and then 0 to not reset SPI
 */
void SPI_DeInint(SPI_RegDef_t *pSPIx) {

	if(pSPIx == SPI1) {
			SPI1_REG_RESET();
		} else if (pSPIx == SPI2) {
			SPI2_REG_RESET();
		} else if (pSPIx == SPI3) {
			SPI3_REG_RESET();
		} else if (pSPIx == SPI4) {
			SPI4_REG_RESET();
		} else if (pSPIx == SPI5) {
			SPI5_REG_RESET();
		} else {
			// wrong port address
		}
}



/**
 * SPI_GetFlagStatus() helper function to get status register values.
 * @param pSPIx             base address of SPIx peripheral
 * @param flagName          name aka SR bit position state check required
 * @param return            status either SET or RESET
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName) {


	if (pSPIx->SR & (1 << flagName))
		return FLAG_SET;
	return FLAG_RESET;
}

/* SPI send and receive data*/

/**
 * SPI_SendData() function sends given number of byte.
 * @param pSPIx             base address of SPIx peripheral
 * @param pTxBuffer         address to buffer containing data to be sent
 * @param length            length of TxBuffer to be sent
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length) {

	while(length > 0){
		// 1. wait until TX buffer empty SR set.
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// Load the 16bit data in to the Data Register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length -= 2;
			(uint16_t*)pTxBuffer++;                            /* Increment the pointer to point next block of 16bit data  */

		} else {
			// Load the 8bit data in to the Data Register
			pSPIx->DR = *pTxBuffer;
			length -= 1;
			pTxBuffer++;				                      /* Increment the pointer to point next block of 8bit data*/
		}

	}
}


void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length) {

}


/* SPI IRQ configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enableOrDisable) {

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

}

/**
 * SPI_PeripheralControl() function enables or disables SPI peripheral.
 * @param pSPIx             base address of SPIx peripheral
 * @param enableOrDisable   boolean 1 to enable/set and 0 to disable/reset
 * Remark: Setting up SPI peripheral requires SPE bit to set 0 and set 1 to apply the changes
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIOx, uint8_t enableOrDisable) {


	if(enableOrDisable == ENABLE) {
		pSPIOx->CR2 |= (1 << SPI_CR2_SSOE);          // Added to fix MODF exception while enabling CR1_SPE. Refer below link
		pSPIOx->CR1 |= (1 << SPI_CR1_SPE);           // causes MODF exception. Refer https://github.com/libopencm3/libopencm3/issues/232
		/* Excerpts from RM of STM32F411CE@20.3.1:
		 * When configured in master mode with NSS configured as an input (MSTR=1 and SSOE=0) and if NSS is pulled low, the SPIc
		  enters the master mode fault state: the MSTR bit is automatically cleared and the device is configured in slave mode*/
	} else {
		pSPIOx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

// Note: SSI bit influence NSS state when SSM=1. By default SSI=0, wo NSS will be pulled low which is not acceptable for Master when working in non multi master situaation.
void SPI_SSIConfig(SPI_RegDef_t *pSPIOx, uint8_t enableOrDisable) {

	if(enableOrDisable == ENABLE) {
		pSPIOx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIOx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
