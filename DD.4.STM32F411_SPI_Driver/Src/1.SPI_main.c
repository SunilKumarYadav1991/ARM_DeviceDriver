/*
 * 1.SPI_main.c
 *
 *  Created on: 17-Jul-2023
 *      Author: Sunil Yadav
 *
 *      Note: SPI testing unsuccesful
 */
#include"stm32f411_spi_driver.h"
#include "stm32f411_gpio_driver.h"         // GPIO related function and macros
#include <string.h>                        // for strlen()

/** Refer Table 9. Alternate function mapping table@Datasheet to identify pin number
* PB15 -> SPI2 MOSI
* PB14 -> SPI2 MISO
* PB13 -> SPI2 SCLK
* PB12 -> SPI2 NSS (slave select)
* Alternate functionality Mode : AF05
*/


/**
 * SPI2_GPIOInit() helper function to initialize the GPIO used for SPI testing.
 * Remark: Using SPI2 as reference
 */
void SPI2_GPIOInit(void) {

	GPIO_Handle_t SPIpins;

	SPIpins.pGPIOx = GPIOB;
	SPIpins.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;          // using pin for SPI functionality
	SPIpins.GPIO_pinConfig.GPIO_PinAltFunMode = 5;                  /*  Alternate functionality Mode : AF05*/
	SPIpins.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIpins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIpins);

	// MISO
	SPIpins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIpins);

	/*  Since this example lacks slave, we can avoid configuring these pins
	// MOSI
	SPIpins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIpins);

	// NSS
	SPIpins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIpins);
	*/
}


/**
 * SPI2_Init() helper function to initialize the SPI2.
 */
void SPI2_Init(void) {

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLOCK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;                      // since no slave device at testing phase, enabling SSM for NSS pin

	SPI_Init(&SPI2handle);
}


int main() {

	// SPI TX testing

	char message[] = "Hello world";

	SPI2_GPIOInit();                                                // Initialize the SPI2 GPIO pin
	SPI2_Init();                                                    // Initializes the SPI relevant parameters for SPI2. Note when SPI is disabled, only then cofig can work

	SPI_SSIConfig(SPI2, ENABLE);                                    // sets NSS signal to high i.e. VDD internally to avoid MODF error

	SPI_PeripheralControl(SPI2, ENABLE);                            // enable the SPI2 peripheral

	SPI_SendData(SPI2, (uint8_t*)message, strlen(message));         // send string data

	while(1);

	return 0;
}

