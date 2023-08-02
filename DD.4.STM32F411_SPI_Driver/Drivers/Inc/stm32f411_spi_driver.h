/*
 * stm32f411_spi_driver.h
 *
 *  Created on: 17-Jul-2023
 *      Author: Sunil Yadav
 */

#ifndef INC_STM32F411_SPI_DRIVER_H_
#define INC_STM32F411_SPI_DRIVER_H_

#include "stm32f411.h"

/**
 * Configuration structure for SPIx peripheral
 */

typedef struct {                                      /* Refer Refer 20.5.1@Refernce manual*/
	uint8_t SPI_DeviceMode;                           /* Device mode, master/slave*/
	uint8_t SPI_BusConfig;                            /* Possible bus mode half/full duplex etc*/
	uint8_t SPI_SclkSpeed;                            /* Serial clock speed aka baud rate*/
	uint8_t SPI_DFF;                                  /* Data frame format. 8 bit or 16 bit */
	uint8_t SPI_CPOL;                                 /* Clock polarity */
	uint8_t SPI_CPHA;                                 /* Clock Phase*/
	uint8_t SPI_SSM;                                  /* Slave select management (sw/internal or hw/external). Possible values from @ */
}SPI_Config_t;


/**
 * This is a Handle struct for a SPI
 */
typedef struct {
	SPI_RegDef_t   *pSPIx;                            /* This holds the base address of the SPIx(x:0-5)peipheral */
	SPI_Config_t SPIConfig;                           /* This holds handle struct for SPI to control the mode etc */

}SPI_Handle_t;

/* @SPI_DeviceMode SPI possible device mode. Refer 20.5.1 bit 2 MSTR*/
#define SPI_DEVICE_MODE_MASTER          1             /* Unless device is in master serial clk is not generated */
#define SPI_DEVICE_MODE_SLAVE           0


/* @SPI_BusConfig SPI possible bus configuration*/
#define SPI_BUS_CONFIG_FD                1            /* Full Duplex*/
#define SPI_BUS_CONFIG_HD                2            /* Half Duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3


/* @SPI_SclkSpeed SPI possible clock speed: Refer 20.5.1 bit 5:3 of reference manual. Ranges are (fz of clock / 2) ..(fz of clock / 256) */
#define SPI_SCLOCK_SPEED_DIV2            0
#define SPI_SCLOCK_SPEED_DIV4            1
#define SPI_SCLOCK_SPEED_DIV8            2
#define SPI_SCLOCK_SPEED_DIV16           3
#define SPI_SCLOCK_SPEED_DIV32           4
#define SPI_SCLOCK_SPEED_DIV64           5
#define SPI_SCLOCK_SPEED_DIV128          6
#define SPI_SCLOCK_SPEED_DIV256          7


/* @SPI_DFF SPI possible frame format*/
#define SPI_DFF_8BITS                    0
#define SPI_DFF_16BITS                   1


/* @SPI_CPOL SPI clock polarity*/
#define SPI_CPOL_LOW                    0
#define SPI_CPOL_HIGH                   1


/* @SPI_CPHA SPI clock phase*/
#define SPI_CPHA_LOW                    0
#define SPI_CPHA_HIGH                   1


/* @SPI_SSM SPI software slave management*/
#define SPI_SSM_DI                      0
#define SPI_SSM_EN                      1




/* SPI Peripheral clock setup*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIOx, uint8_t enableOrDisable);

/* SPI Init and De-Init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInint(SPI_RegDef_t *pSPIx);              /* RCC AHB1/ABP peripheral reset register can be used to reset the GPIOx state to reset state */


/* SPI send and receive data*/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length);


/* SPI IRQ configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enableOrDisable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/* Function to enble SPI communication. Refer CR1.SPE*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIOx, uint8_t enableOrDisable);

void SPI_SSIConfig(SPI_RegDef_t *pSPIOx, uint8_t enableOrDisable);                       /* configure SSI to avoid faults. Refer reference manual*/

#endif /* INC_STM32F411_SPI_DRIVER_H_ */
