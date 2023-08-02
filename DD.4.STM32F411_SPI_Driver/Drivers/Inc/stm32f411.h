/*
 * stm32f411.h
 *
 *   Created on: 04-Feb-2023
 *       Author: Sunil Yadav
 *  Description: STM32F411 peripheral register macro
 *  Updates: Added SPI related register definitions
 */

#ifndef INC_STM32F411_H_
#define INC_STM32F411_H_

#include <stdint.h>

/***************************************** ARM Cortex M4 Specific macro ***********************************************************/


#define FLASH_BASEADDR                   0x08000000U                              /* Start address of 512K flash*/
#define SRAM1_BASEADDR                   0x20000000U                              /* Start address of 128K SRAM*/
#define SRAM                             SRAM1_BASEADDR                           /* This controller have only one SRAM and there is no aux/SRAM2*/
#define ROM_BASEADDR                     0x1FFF0000U                              /* Start address of 30K ROM */
#define OTP_BASEADDR                     0x1FFF7800U                              /* Start address of 528 byte OneTimeProgrmmable area*/



/********************** ARM Cortex Mx NVIC Interrupt Set Enable Register addresses@Section 4.3 Generic Cortex M4 Manual *****************************************/
#define NVIC_ISER0                     ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1                     ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2                     ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3                     ((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4                     ((volatile uint32_t*)0xE000E110)
#define NVIC_ISER5                     ((volatile uint32_t*)0xE000E114)
#define NVIC_ISER6                     ((volatile uint32_t*)0xE000E118)
#define NVIC_ISER7                     ((volatile uint32_t*)0xE000E11C)



/* Macro for few of the possible IRQ priority no. */
#define NVIC_IRQ_PRI_0                 0
#define NVIC_IRQ_PRI_1                 1
#define NVIC_IRQ_PRI_2                 2
#define NVIC_IRQ_PRI_3                 3
#define NVIC_IRQ_PRI_4                 4
#define NVIC_IRQ_PRI_5                 5
#define NVIC_IRQ_PRI_6                 6
#define NVIC_IRQ_PRI_7                 7
#define NVIC_IRQ_PRI_8                 8
#define NVIC_IRQ_PRI_9                 9
#define NVIC_IRQ_PRI_10                10
#define NVIC_IRQ_PRI_11                11
#define NVIC_IRQ_PRI_12                12
#define NVIC_IRQ_PRI_13                13
#define NVIC_IRQ_PRI_14                14
#define NVIC_IRQ_PRI_15                15
#define NVIC_IRQ_PRI_16                16
#define NVIC_IRQ_PRI_17                17


/********************* ARM Cortex Mx NVIC Interrupt Clear Enable Register addresses@Section 4.3 Generic Cortex M4 Manual ***************************************/
#define NVIC_ICER0                     ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1                     ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2                     ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3                     ((volatile uint32_t*)0xE000E18C)
#define NVIC_ICER4                     ((volatile uint32_t*)0xE000E190)
#define NVIC_ICER5                     ((volatile uint32_t*)0xE000E194)
#define NVIC_ICER6                     ((volatile uint32_t*)0xE000E198)
#define NVIC_ICER7                     ((volatile uint32_t*)0xE000E19C)


/* Interrupt priority register x (NVIC_IPRX0 - NVIC_IPR59). Refer Section 4.3.7@Generic Cortex M4 manual*/
#define NVIC_PRIORITY_BASE_ADDR       ((volatile uint32_t*)0xE000E400)        /* Base address of NVIC priority register*/


#define NUMBER_OF_PRIORITY_BIT_IMPLEMENTED     4                             /* Refer Table 47@Generic Cort bits[7:4] are implemented in STM32F4* and bits[3:0] are ignored*/
/***************************************** Peripheral base addresses ***********************************************************/

/* Peripheral base addresses i.e. base addresses of peripherals attached to different buses*/
/* Refer Reference Manual 2.3 Memory Map@Refernce Manual*/
#define PERIPH_BASE                       0x40000000U                              /* Start address of Peripheral*/
#define APB1PERIPH_BASE                   PERIPH_BASE                              /* Start address of Peripheral attached to APB1*/
#define APB2PERIPH_BASE                   0x40010000U                              /* Start address of Peripheral attached to APB2*/
#define AHB1PERIPH_BASE                   0x40020000U                              /* Start address of Peripheral attached to AHB1*/
#define AHB2PERIPH_BASE                   0x50000000U                              /* Start address of Peripheral attached to AHB2*/

/* Base address of peripheral attached to AHB1 bus. We've skipped few for the time being*/
#define GPIOA_BASEADDR                    (AHB1PERIPH_BASE+0x0000)                  /* Start address of GPIO A*/
#define GPIOB_BASEADDR                    (AHB1PERIPH_BASE+0x0400)                  /* Start address of GPIO B*/
#define GPIOC_BASEADDR                    (AHB1PERIPH_BASE+0x0800)                  /* Start address of GPIO C*/
#define GPIOD_BASEADDR                    (AHB1PERIPH_BASE+0x0C00)                  /* Start address of GPIO D*/
#define GPIOE_BASEADDR                    (AHB1PERIPH_BASE+0x1000)                  /* Start address of GPIO E*/
#define GPIOH_BASEADDR                    (AHB1PERIPH_BASE+0x1C00)                  /* Start address of GPIO F*/
#define RCC_BASEADDR                      (AHB1PERIPH_BASE+0x3800)                  /* Start address ofRCC*/

/* Base address of peripheral attached to APB1 bus. We've skipped few for the time being*/
#define I2C1_BASEADDR                      (APB1PERIPH_BASE+0x5400)                   /* Start address of I2C 1*/
#define I2C2_BASEADDR                      (APB1PERIPH_BASE+0x5800)                   /* Start address of I2C 2*/
#define I2C3_BASEADDR                      (APB1PERIPH_BASE+0x5C00)                   /* Start address of I2C 3*/
#define USART2_BASEADDR                    (APB1PERIPH_BASE+0x4400)                   /* Start address of USART 2*/
#define SPI2_BASEADDR                      (APB1PERIPH_BASE+0x3800)                   /* Start address of SPI 2*/
#define SPI3_BASEADDR                      (APB1PERIPH_BASE+0x3C00)                   /* Start address of SPI 3*/


/* Base address of peripheral attached to APB2 bus. We've skipped few for the time being*/
#define USART1_BASEADDR                    (APB2PERIPH_BASE+0x1000)                   /* Start address of USART 1*/
#define USART6_BASEADDR                    (APB2PERIPH_BASE+0x1400)                   /* Start address of USART 6*/
#define ADC1_BASEADDR                      (APB2PERIPH_BASE+0x2000)                   /* Start address of ADC 1*/
#define SPI1_BASEADDR                      (APB2PERIPH_BASE+0x3000)                   /* Start address of SPI 1*/
#define SPI4_BASEADDR                      (APB2PERIPH_BASE+0x3400)                   /* Start address of SPI 4*/
#define SPI5_BASEADDR                      (APB2PERIPH_BASE+0x5000)                   /* Start address of SPI 5*/
#define SYSCFG_BASEADDR                    (APB2PERIPH_BASE+0x3800)                   /* Start address of SYSCFG i.e. System Configuration Controller*/
#define EXTI_BASEADDR                      (APB2PERIPH_BASE+0x3C00)                   /* Start address of EXTI i.e. external interrupt*/


/***************************************** GPIO peripheral register definition structure ***********************************************************/
typedef struct {                                                                        // Refer 8.4.11 GPIO register map for more details @Reference manual
	volatile uint32_t MODER;                                                            /* GPIO port mode register*/
	volatile uint32_t OTYPER;                                                           /* GPIO port output type register*/
	volatile uint32_t OSPEEDR;                                                          /* GPIO port output speed register*/
	volatile uint32_t PUPDR;                                                            /* GPIO port pull-up/pull-down register*/
	volatile uint32_t IDR;                                                              /* GPIO port input data register*/
	volatile uint32_t ODR;                                                              /* GPIO port output data register*/
	volatile uint32_t BSRR;                                                             /* GPIO port bit set/reset register*/
	volatile uint32_t LCKR;                                                             /* GPIO port configuration lock register*/
	volatile uint32_t AFRL;                                                             /* GPIO alternate function low register*/
	volatile uint32_t AFRH;                                                             /* GPIO alternate function high register*/
}GPIO_RegDef_t;



/***************************************** RCC peripheral register definition structure ***********************************************************/
typedef struct {                                                                         // Refer 6.3.22 RCC register map @Reference manual
	volatile uint32_t CR;                                                                /* RCC clock control register*/
	volatile uint32_t PLLCFGR;                                                           /* RCC PLL configuration register*/
	volatile uint32_t CFGR;                                                              /* RCC clock configuration register*/
	volatile uint32_t CIR;                                                               /* RCC clock interrupt register*/
	volatile uint32_t AHB1RSTR;                                                          /* RCC AHB1 peripheral reset register*/
	volatile uint32_t AHB2RSTR;                                                          /* RCC AHB2 peripheral reset register*/
	volatile uint32_t RESERVED1;                                                         /* RESERVED1*/
	volatile uint32_t RESERVED2;                                                         /* RESERVED1*/
	volatile uint32_t APB1RSTR;                                                          /* RCC APB1 peripheral reset register*/
	volatile uint32_t APB2RSTR;                                                          /* RCC APB2 peripheral reset register*/
	volatile uint32_t RESERVED3;                                                         /* RESERVED 3*/
	volatile uint32_t RESERVED4;                                                         /* RESERVED 4*/
	volatile uint32_t AHB1ENR;                                                           /* RCC AHB1 peripheral clock enable register*/
	volatile uint32_t AHB2ENR;                                                           /* RCC AHB2 peripheral clock enable register*/
	volatile uint32_t RESERVED5;                                                         /* RESERVED 5*/
	volatile uint32_t RESERVED6;                                                         /* RESERVED 6*/
	volatile uint32_t APB1ENR;                                                           /* RCC APB1 peripheral clock enable register*/
	volatile uint32_t APB2ENR;                                                           /* RCC APB2 peripheral clock enable register*/
	volatile uint32_t RESERVED7;                                                         /* RESERVED 7*/
	volatile uint32_t RESERVED8;                                                         /* RESERVED 8*/
	volatile uint32_t AHB1LPENR;                                                         /* RCC AHB1 peripheral clock enable in low power mode register*/
	volatile uint32_t AHB2LPENR;                                                         /* RCC AHB2 peripheral clock enable in low power mode register*/
	volatile uint32_t RESERVED9;                                                         /* RESERVED 9*/
	volatile uint32_t RESERVED10;                                                        /* RESERVED 10*/
	volatile uint32_t APB1LPENR;                                                         /* RCC APB1 peripheral clock enable in low power mode register*/
	volatile uint32_t APB2LPENR;                                                         /* RCC APB2 peripheral clock enable in low power mode register*/
	volatile uint32_t RESERVED11;                                                        /* RESERVED 11*/
	volatile uint32_t RESERVED12;                                                        /* RESERVED v12*/
	volatile uint32_t BDCR;                                                              /* RCC Backup domain control register*/
	volatile uint32_t CSR;                                                               /* RCC clock control & status register*/
	volatile uint32_t RESERVED13;                                                        /* RESERVED 13*/
	volatile uint32_t RESERVED14;                                                        /* RESERVED 14*/
	volatile uint32_t SSCGR;                                                             /* RCC spread spectrum clock generation register*/
	volatile uint32_t PLLI2SCFGR;                                                        /* RCC PLLI2S configuration register*/
	volatile uint32_t DCKCFGR;                                                           /* RCC Dedicated Clocks Configuration Register*/
}RCC_RegDef_t;


/***************************************** EXTI peripheral register definition structure ***********************************************************/
typedef struct {                                                                          // Refer 10.3.7 EXTI register map @Reference manual
	volatile uint32_t IMR;                                                                /* EXTI Interrupt mask register*/
	volatile uint32_t EMR;                                                                /* EXTI Event mask register*/
	volatile uint32_t RTSR;                                                               /* EXTI Rising trigger selection register*/
	volatile uint32_t FTSR;                                                               /* EXTI Falling trigger selection register*/
	volatile uint32_t SWIER;                                                              /* EXTI Software interrupt event register*/
	volatile uint32_t PR;                                                                 /* EXTI Pending register*/
}EXTI_RegDef_t;


/***************************************** SYSCFG peripheral register definition structure ***********************************************************/
typedef struct {                                                                          // Refer 7.2.8 SYSCFG register map @Reference manual
	volatile uint32_t MEMRMP;                                                             /* SYSCFG memory remap register*/
	volatile uint32_t PMC;                                                                /* SYSCFG peripheral mode configuration register*/
	volatile uint32_t EXTICR[4];                                                          /* SYSCFG external interrupt configuration register 1-4*/
	volatile uint32_t CMPCR;                                                              /* SYSCFG Compensation cell control register*/
}SYSCFG_RegDef_t;


/***************************************** SPI and I2S peripheral register definition structure ***********************************************************/
typedef struct {                                                                          // Refer 20.5.10 SPI register map @Reference manual
	volatile uint32_t CR1;                                                               /* SPI control register 1 (SPI_CR1)(not used in I2S mode)*/
	volatile uint32_t CR2;                                                               /* SPI control register 2 (SPI_CR2) */
	volatile uint32_t SR;                                                                /* SPI status register (SPI_SR)*/
	volatile uint32_t DR;                                                                /* SPI data register (SPI_DR)r*/
	volatile uint32_t CRCPR;                                                             /* SPI CRC polynomial register (SPI_CRCPR)(not used in I2S
mode)*/
	volatile uint32_t RXCRCR;                                                            /* SPI RX CRC register (SPI_RXCRCR)(not used in I2S mode)*/
	volatile uint32_t TXCRCR;                                                            /* SPI TX CRC register (SPI_TXCRCR)(not used in I2S mode)*/
	volatile uint32_t I2SCFGR;                                                           /* SPI_I2S configuration register (SPI_I2SCFGR)*/
	volatile uint32_t I2SPR;                                                             /* SPI_I2S prescaler register (SPI_I2SPR)*/

}SPI_RegDef_t;


#define GPIOA                               ((GPIO_RegDef_t*)GPIOA_BASEADDR)            /* GPIO A base address type casted to register definition structure*/
#define GPIOB                               ((GPIO_RegDef_t*)GPIOB_BASEADDR)            /* GPIO B base address type casted to register definition structure*/
#define GPIOC                               ((GPIO_RegDef_t*)GPIOC_BASEADDR)            /* GPIO C base address type casted to register definition structure*/
#define GPIOD                               ((GPIO_RegDef_t*)GPIOD_BASEADDR)            /* GPIO D base address type casted to register definition structure*/
#define GPIOE                               ((GPIO_RegDef_t*)GPIOE_BASEADDR)            /* GPIO E base address type casted to register definition structure*/
#define GPIOH                               ((GPIO_RegDef_t*)GPIOH_BASEADDR)            /* GPIO H base address type casted to register definition structure*/

#define RCC                                 ((RCC_RegDef_t*)RCC_BASEADDR)               /* RCC base address type casted to register definition structure*/

#define EXTI                                ((EXTI_RegDef_t*)EXTI_BASEADDR)             /* EXTI i.e External Interrupt base address type casted to register definition structure*/

#define SYSCFG                              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)         /* EXTI i.e External Interrupt base address type casted to register definition structure*/



#define SPI1                               ((SPI_RegDef_t*)SPI1_BASEADDR)              /* SPI 1 base address type casted to register definition structure*/
#define SPI2                               ((SPI_RegDef_t*)SPI2_BASEADDR)              /* SPI 2 base address type casted to register definition structure*/
#define SPI3                               ((SPI_RegDef_t*)SPI3_BASEADDR)              /* SPI 3 base address type casted to register definition structure*/
#define SPI4                               ((SPI_RegDef_t*)SPI4_BASEADDR)              /* SPI 4 base address type casted to register definition structure*/
#define SPI5                               ((SPI_RegDef_t*)SPI5_BASEADDR)              /* SPI 5 base address type casted to register definition structure*/



/* Clock enable macros for GPIOx peripherals*/
#define GPIOA_PCLK_EN()           (RCC->AHB1ENR |= (1<<0) )                   /* Enable GPIOA by setting bit 0 1*/
#define GPIOB_PCLK_EN()           (RCC->AHB1ENR |= (1<<1) )                   /* Enable GPIOB by setting bit 1 1*/
#define GPIOC_PCLK_EN()           (RCC->AHB1ENR |= (1<<2) )                   /* Enable GPIOC by setting bit 2 1*/
#define GPIOD_PCLK_EN()           (RCC->AHB1ENR |= (1<<3) )                   /* Enable GPIOD by setting bit 3 1*/
#define GPIOE_PCLK_EN()           (RCC->AHB1ENR |= (1<<4) )                   /* Enable GPIOE by setting bit 4 1*/
#define GPIOH_PCLK_EN()           (RCC->AHB1ENR |= (1<<7) )                   /* Enable GPIOH by setting bit 7 1*/


/* Clock enable macros for I2Cx peripherals*/
#define I2C1_PCLK_EN()           (RCC->APB1ENR |= (1<<21) )                   /* Enable I2C1 by setting bit 21 to 1*/
#define I2C2_PCLK_EN()           (RCC->APB1ENR |= (1<<22) )                   /* Enable I2C2 by setting bit 22 to 1*/
#define I2C3_PCLK_EN()           (RCC->APB1ENR |= (1<<23) )                   /* Enable I2C3 by setting bit 23 to 1*/


/* Clock enable macros for SPIx peripherals*/
#define SPI1_PCLK_EN()           (RCC->APB2ENR |= (1<<12) )                   /* Enable SPI1 by setting APB2ENR bit 12 to 1*/
#define SPI2_PCLK_EN()           (RCC->APB1ENR |= (1<<14) )                   /* Enable SPI2 by setting APB1ENR bit 14 to 1*/
#define SPI3_PCLK_EN()           (RCC->APB1ENR |= (1<<15) )                   /* Enable SPI3 by setting APB2ENR bit 15 to 1*/
#define SPI4_PCLK_EN()           (RCC->APB2ENR |= (1<<13) )                   /* Enable SPI3 by setting APB2ENR bit 13 to 1*/
#define SPI5_PCLK_EN()           (RCC->APB2ENR |= (1<<20) )                   /* Enable SPI3 by setting APB2ENR bit 20 to 1*/


/* Clock enable macros for USARTx peripherals*/
#define USART1_PCLK_EN()           (RCC->APB2ENR |= (1<<4) )                   /* Enable USART 1 by setting APB2ENR bit 4 to 1*/
#define USART2_PCLK_EN()           (RCC->APB1ENR |= (1<<17) )                  /* Enable USART 2 by setting APB1ENR bit 17 to 1*/
#define USART6_PCLK_EN()           (RCC->APB2ENR |= (1<<5) )                   /* Enable USART 6 by setting APB2ENR bit 5 to 1*/


/* Clock enable macros for SYSCFG peripherals*/
#define SYSCFG_PCLK_EN()           (RCC->APB2ENR |= (1<<14) )                 /* Enable System configuration controller clock by setting APB2ENR bit 14 to 1*/


/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* Clock Disable macros for GPIOx peripherals*/
#define GPIOA_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<0) )                   /* Disable GPIOA by setting bit 0 0*/
#define GPIOB_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<1) )                   /* Disable GPIOB by setting bit 1 0*/
#define GPIOC_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<2) )                   /* Disable GPIOC by setting bit 2 0*/
#define GPIOD_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<3) )                   /* Disable GPIOD by setting bit 3 0*/
#define GPIOE_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<4) )                   /* Disable GPIOE by setting bit 4 0*/
#define GPIOH_PCLK_DI()           (RCC->AHB1ENR &= ~(1<<7) )                   /* Disable GPIOH by setting bit 7 0*/


/* Clock Disable macros for I2Cx peripherals*/
#define I2C1_PCLK_DI()           (RCC->APB1ENR &= ~(1<<21) )                   /* Disable I2C1 by setting bit 21 to 0*/
#define I2C2_PCLK_DI()           (RCC->APB1ENR &= ~(1<<22) )                   /* Disable I2C2 by setting bit 22 to 0*/
#define I2C3_PCLK_DI()           (RCC->APB1ENR &= ~(1<<23) )                   /* Disable I2C3 by setting bit 23 to 0*/


/* Clock Disable macros for SPIx peripherals*/
#define SPI1_PCLK_DI()           (RCC->APB2ENR &= ~(1<<12) )                   /* Disable SPI1 by setting APB2ENR bit 12 to 0*/
#define SPI2_PCLK_DI()           (RCC->APB1ENR &= ~(1<<14) )                   /* Disable SPI2 by setting APB1ENR bit 14 to 0*/
#define SPI3_PCLK_DI()           (RCC->APB1ENR &= ~(1<<15) )                   /* Disable SPI3 by setting APB2ENR bit 15 to 0*/
#define SPI4_PCLK_DI()           (RCC->APB2ENR &= ~(1<<13) )                   /* Disable SPI3 by setting APB2ENR bit 13 to 0*/
#define SPI5_PCLK_DI()           (RCC->APB2ENR &= ~(1<<20) )                   /* Disable SPI3 by setting APB2ENR bit 20 to 0*/


/* Clock Disable macros for USARTx peripherals*/
#define USART1_PCLK_DI()           (RCC->APB2ENR &= ~(1<<4) )                   /* Disable USART 1 by setting APB2ENR bit 4 to 0*/
#define USART2_PCLK_DI()           (RCC->APB1ENR &= ~(1<<17) )                  /* Disable USART 2 by setting APB1ENR bit 17 to 0*/
#define USART6_PCLK_DI()           (RCC->APB2ENR &= ~(1<<5) )                   /* Disable USART 6 by setting APB2ENR bit 5 to 0*/


/* Clock Disable macros for SYSCFG peripherals*/
#define SYSCFG_PCLK_DI()           (RCC->APB2ENR &= ~(1<<14) )                   /* Disable System configuration controller clock by setting APB2ENR bit 14 to 0*/


/* Macros to reset GPIO Peripherals. Refer 6.3.5 RCC AHB1 peripheral reset register (RCC_AHB1RSTR) @Refernce manual. To reset we need to reset(1) and revert to normal state(0) */
#define GPIOA_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<0) );  (RCC->AHB1RSTR &= ~(1<<0) ); }while(0)
#define GPIOB_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<1) );  (RCC->AHB1RSTR &= ~(1<<1) ); }while(0)
#define GPIOC_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<2) );  (RCC->AHB1RSTR &= ~(1<<2) ); }while(0)
#define GPIOD_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<3) );  (RCC->AHB1RSTR &= ~(1<<3) ); }while(0)
#define GPIOE_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<4) );  (RCC->AHB1RSTR &= ~(1<<4) ); }while(0)
#define GPIOH_REG_RESET()           do { (RCC->AHB1RSTR |= (1<<7) );  (RCC->AHB1RSTR &= ~(1<<7) ); }while(0)


/* Macro to return code required to write into SYSCFG EXTI Control Register. TODO: Convert this macro into simple function */
#define GPIO_BASEADDR_TO_CODE(portAddress)              ( (portAddress == GPIOA)? 0:\
		                                                  (portAddress == GPIOB)? 1:\
		                                          		  (portAddress == GPIOC)? 2:\
				                                          (portAddress == GPIOD)? 3:\
						                                  (portAddress == GPIOE)? 4:\
								                          (portAddress == GPIOH)? 7:0)



/* Interrupt Request Number (IRQ) of EXTI@STM32F411xC/E . Refer Table 32 @Reference Manual*/
#define IRQ_NO_EXTI0                6                                      /* EXTI Line 0 interrupts*/
#define IRQ_NO_EXTI1                7                                      /* EXTI Line 1 interrupts*/
#define IRQ_NO_EXTI2                8                                      /* EXTI Line 2 interrupts*/
#define IRQ_NO_EXTI3                9                                      /* EXTI Line 3 interrupts*/
#define IRQ_NO_EXTI4                10                                     /* EXTI Line 4 interrupts*/
#define IRQ_NO_EXTI9_5              23                                     /* EXTI Line[9:5] interrupts*/
#define IRQ_NO_EXTI15_10            40                                     /* EXTI Line[15:10] interrupts*/
/* Note: IRQ_NO for EXTI 16, 17, 21, 22 not included due to multi usage in vector table */

/* Interrupt Request Number (IRQ) of SPI@STM32F411xC/E . Refer Table XX @Reference Manual*/ // Pending



/* Macros to reset SPI x Peripherals. Refer 6.3.7/8 RCC APB1 APB2 peripheral reset register (RCC_APB1RSTR & RCC_APB2RSTR) @Refernce manual. To reset we need to reset(1) and revert to normal state(0) */
#define SPI1_REG_RESET()           do { (RCC->APB2RSTR |= (1<<12) );  (RCC->APB2RSTR &= ~(1<<12) ); }while(0)
#define SPI2_REG_RESET()           do { (RCC->APB1RSTR |= (1<<14) );  (RCC->APB1RSTR &= ~(1<<14) ); }while(0)
#define SPI3_REG_RESET()           do { (RCC->APB1RSTR |= (1<<15) );  (RCC->APB1RSTR &= ~(1<<15) ); }while(0)
#define SPI4_REG_RESET()           do { (RCC->APB2RSTR |= (1<<13) );  (RCC->APB2RSTR &= ~(1<<13) ); }while(0)
#define SPI5_REG_RESET()           do { (RCC->APB2RSTR |= (1<<20) );  (RCC->APB2RSTR &= ~(1<<20) ); }while(0)



/***************************************** Miscellaneous Macros ***********************************************************/
#define ENABLE                    1
#define DISABLE                   0
#define SET                       ENABLE
#define RESET                     DISABLE

#define GPIO_PIN_SET              SET
#define GPIO_PIN_RESET            RESET

#define FLAG_RESET                RESET
#define FLAG_SET                  SET

/***************************************** SPI CR1 Bit Macros ***********************************************************/
#define SPI_CR1_CPHA              0
#define SPI_CR1_CPOL              1
#define SPI_CR1_MSTR              2
#define SPI_CR1_BR                3       /* bit 3:5*/
#define SPI_CR1_SPE               6
#define SPI_CR1_LSB_FIRST         7
#define SPI_CR1_SSI               8
#define SPI_CR1_SSM               9
#define SPI_CR1_RX_ONLY           10
#define SPI_CR1_DFF               11
#define SPI_CR1_CRC_NEXT          12
#define SPI_CR1_CRC_EN            13
#define SPI_CR1_BIDI_OE           14
#define SPI_CR1_BIDI_MODE         15


/***************************************** SPI CR2 Bit Macros ***********************************************************/
#define SPI_CR2_RXDMAEN           0
#define SPI_CR2_TXDMAEN           1
#define SPI_CR2_SSOE              2
#define SPI_CR2_FRF               4
#define SPI_CR2_ERRIE             5
#define SPI_CR2_RXNEIE            6
#define SPI_CR2_TXEIE             7


/***************************************** SPI SR Bit Macros ***********************************************************/
#define SPI_SR_RXNE               0
#define SPI_SR_TXE                1
#define SPI_SR_CHSIDE             2
#define SPI_SR_UDR                3
#define SPI_SR_CRCERR             4
#define SPI_SR_MODF               5
#define SPI_SR_OVR                6
#define SPI_SR_BSY                7
#define SPI_SR_FRE                8



#endif /* INC_STM32F411_H_ */
