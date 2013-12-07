/*
 * (C) Copyright 2011
 *
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * STM32 processor definitions
 */
#ifndef _MACH_STM32_H_
#define _MACH_STM32_H_


/******************************************************************************
 * Global Macros
 ******************************************************************************/
#ifndef NOP
#define NOP()	asm("nop")
#endif

/******************************************************************************
 * Peripheral memory map
 ******************************************************************************/

#define STM32_FLASH_BASE    	((uint32_t)0x08000000) 		/*!< FLASH(up to 1 MB) base address in the alias region                         */
#define STM32_CCMDATARAM_BASE  	((uint32_t)0x10000000) 		/*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define STM32_SRAM1_BASE       	((uint32_t)0x20000000) 		/*!< SRAM1(112 KB) base address in the alias region                             */
#define STM32_SRAM2_BASE       	((uint32_t)0x2001C000) 		/*!< SRAM2(16 KB) base address in the alias region                              */
#define STM32_SRAM3_BASE       	((uint32_t)0x20020000) 		/*!< SRAM3(64 KB) base address in the alias region                              */
#define STM32_PERIPH_BASE      	((uint32_t)0x40000000) 		/*!< Peripheral base address in the alias region                                */
#define STM32_BKPSRAM_BASE     	((uint32_t)0x40024000) 		/*!< Backup SRAM(4 KB) base address in the alias region                         */

#define CCMDATARAM_BB_BASE     	((uint32_t)0x12000000) 		/*!< CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region  */
#define SRAM1_BB_BASE          	((uint32_t)0x22000000) 		/*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB_BASE          	((uint32_t)0x2201C000) 		/*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB_BASE          	((uint32_t)0x22400000) 		/*!< SRAM3(64 KB) base address in the bit-band region                              */
#define PERIPH_BB_BASE         	((uint32_t)0x42000000) 		/*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE        	((uint32_t)0x42024000) 		/*!< Backup SRAM(4 KB) base address in the bit-band region                         */

#define STM32_APB1PERIPH_BASE 	STM32_PERIPH_BASE
#define STM32_APB2PERIPH_BASE 	(STM32_PERIPH_BASE + 0x00010000)
#define STM32_AHB1PERIPH_BASE  	(STM32_PERIPH_BASE + 0x00020000)
#define STM32_AHB2PERIPH_BASE  	(STM32_PERIPH_BASE + 0x10000000)


/******************************************************************************
 * Reset and Clock Control
 ******************************************************************************/

/*
 * RCC register map
 */
struct stm32_rcc_regs {
	uint32_t cr;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	uint32_t pllcfgr;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	uint32_t cfgr;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	uint32_t cir;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	uint32_t ahb1rstr;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	uint32_t ahb2rstr;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	uint32_t ahb3rstr;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t reserved0;     /*!< Reserved, 0x1C                                                                    */
	uint32_t apb1rstr;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	uint32_t apb2rstr;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t reserved1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	uint32_t ahb1enr;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	uint32_t ahb2enr;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	uint32_t ahb3enr;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t reserved2;     /*!< Reserved, 0x3C                                                                    */
	uint32_t apb1enr;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	uint32_t apb2enr;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t reserved3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	uint32_t ahb1lpenr;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	uint32_t ahb2lpenr;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	uint32_t ahb3lpenr;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t reserved4;     /*!< Reserved, 0x5C                                                                    */
	uint32_t apb1lpenr;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	uint32_t apb2lpenr;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t reserved5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	uint32_t bdcr;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	uint32_t csr;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t reserved6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	uint32_t sscgr;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	uint32_t plli2scfgr;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	uint32_t pllsaicfgr;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
	uint32_t dckcfgr;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
};

/*
 * Clocks enumeration
 */
enum clock {
	CLOCK_SYSCLK,		/* SYSCLK clock frequency expressed in Hz     */
	CLOCK_HCLK,			/* HCLK clock frequency expressed in Hz       */
	CLOCK_PCLK1,		/* PCLK1 clock frequency expressed in Hz      */
	CLOCK_PCLK2,		/* PCLK2 clock frequency expressed in Hz      */
	CLOCK_SYSTICK,		/* Systimer clock frequency expressed in Hz   */
	CLOCK_END			/* for internal usage			      */
};

/*
 * RCC registers base
 */
#define STM32_RCC_BASE			(STM32_AHB1PERIPH_BASE + 0x3800)
#define STM32_RCC				((volatile struct stm32_rcc_regs *)STM32_RCC_BASE)

#define STM32_RCC_OFFSET     	(STM32_RCC_BASE - STM32_PERIPH_BASE)
#define STM32_CR_OFFSET         (STM32_RCC_OFFSET + 0x00)

/* Alias word address of PLLSAION bit */
#define PLLSAION_BitNumber   	0x1C
#define CR_PLLSAION_BB         (PERIPH_BB_BASE + (STM32_CR_OFFSET * 32) + (PLLSAION_BitNumber * 4))

/******************************************************************************
 * FIXME: get rid of this
 ******************************************************************************/

/*
 * Return a clock value for the specified clock.
 * Note that we need this function in RAM because it will be used
 * during self-upgrade of U-boot into eNMV.
 * @param clck          id of the clock
 * @returns             frequency of the clock
 */
unsigned long  __attribute__((section(".ramcode")))
	       __attribute__ ((long_call))
	       clock_get(enum clock clck);

#define PAGE_SIZE	4096


/******************************************************************************
 * Flexible Memory Controller Bank5_6
 ******************************************************************************/

struct stm32_fmc_dram_regs {	
	uint32_t sdcr[2];   	/*!< SDRAM Control registers ,     Address offset: 0x140-0x144  */
	uint32_t sdtr[2];     	/*!< SDRAM Timing registers ,      Address offset: 0x148-0x14C  */
  	uint32_t sdcmr;       	/*!< SDRAM Command Mode register,  Address offset: 0x150  */
  	uint32_t sdrtr;       	/*!< SDRAM Refresh Timer register, Address offset: 0x154  */
  	uint32_t sdsr;        	/*!< SDRAM Status register,        Address offset: 0x158  */
};

/*
 * FMC Bank 5/6 registers base
 */
#define STM32_FMC_BASE         	((uint32_t)0xA0000000)
#define STM32_FMC_DRAM_BASE		(STM32_FMC_BASE + 0x0140)
#define STM32_FMC_DRAM			((volatile struct stm32_fmc_dram_regs *)STM32_FMC_DRAM_BASE)


/******************************************************************************
 * LCD-TFT Display Controller
 ******************************************************************************/

struct stm32_ltdc_regs {
	uint32_t reserved0[2];  /*!< Reserved, 0x00-0x04 */
	uint32_t sscr;          /*!< LTDC Synchronization Size Configuration Register,    Address offset: 0x08 */
  	uint32_t bpcr;          /*!< LTDC Back Porch Configuration Register,              Address offset: 0x0C */
  	uint32_t awcr;          /*!< LTDC Active Width Configuration Register,            Address offset: 0x10 */
  	uint32_t twcr;          /*!< LTDC Total Width Configuration Register,             Address offset: 0x14 */
  	uint32_t gcr;           /*!< LTDC Global Control Register,                        Address offset: 0x18 */
  	uint32_t reserved1[2];  /*!< Reserved, 0x1C-0x20 */
  	uint32_t srcr;          /*!< LTDC Shadow Reload Configuration Register,           Address offset: 0x24 */
  	uint32_t reserved2[1];  /*!< Reserved, 0x28 */
  	uint32_t bccr;          /*!< LTDC Background Color Configuration Register,        Address offset: 0x2C */
  	uint32_t reserved3[1];  /*!< Reserved, 0x30 */
  	uint32_t ier;           /*!< LTDC Interrupt Enable Register,                      Address offset: 0x34 */
  	uint32_t isr;           /*!< LTDC Interrupt Status Register,                      Address offset: 0x38 */
  	uint32_t icr;           /*!< LTDC Interrupt Clear Register,                       Address offset: 0x3C */
  	uint32_t lipcr;         /*!< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40 */
  	uint32_t cpsr;          /*!< LTDC Current Position Status Register,               Address offset: 0x44 */
  	uint32_t cdsr;          /*!< LTDC Current Display Status Register,                       Address offset: 0x48 */
};

/*
 * LCD-TFT Display Controller base
 */
#define STM32_LTDC_BASE       	(STM32_APB2PERIPH_BASE + 0x6800)
#define STM32_LTDC				((volatile struct stm32_ltdc_regs *)STM32_LTDC_BASE)

/******************************************************************************
 * LCD-TFT Display layer x Controller
 ******************************************************************************/
struct stm32_ltdc_layer_regs
{
  	uint32_t cr;            /*!< LTDC Layerx Control Register                                  Address offset: 0x84 */
  	uint32_t whpcr;         /*!< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88 */
  	uint32_t wvpcr;         /*!< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C */
  	uint32_t ckcr;          /*!< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90 */
  	uint32_t pfcr;          /*!< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94 */
  	uint32_t cacr;          /*!< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98 */
  	uint32_t dccr;          /*!< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C */
  	uint32_t bfcr;          /*!< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0 */
  	uint32_t reserved0[2];  /*!< Reserved */
  	uint32_t cfbar;         /*!< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC */
  	uint32_t cfblr;         /*!< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0 */
  	uint32_t cfblnr;        /*!< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4 */
  	uint32_t reserved1[3];  /*!< Reserved */
  	uint32_t clutwr;        /*!< LTDC Layerx CLUT Write Register                               Address offset: 0x144 */
};

/*
 * LCD-TFT Display Layer x Controller base
 */
#define STM32_LTDC_LAYER1_BASE 		(STM32_LTDC_BASE + 0x084)
#define STM32_LTDC_LAYER2_BASE 		(STM32_LTDC_BASE + 0x104)
#define STM32_LTDC_LAYER1			((volatile struct stm32_ltdc_layer_regs *)STM32_LTDC_LAYER1_BASE)
#define STM32_LTDC_LAYER2			((volatile struct stm32_ltdc_layer_regs *)STM32_LTDC_LAYER2_BASE)


/******************************************************************************
 * Power Control
 ******************************************************************************/

struct stm32_pwr_regs {
  	uint32_t cr;   			/*!< PWR power control register,        Address offset: 0x00 */
  	uint32_t csr;  			/*!< PWR power control/status register, Address offset: 0x04 */
};

/*
 * Power control registers base
 */
#define STM32_PWR_BASE       	(STM32_APB1PERIPH_BASE + 0x7000)
#define STM32_PWR				((volatile struct stm32_pwr_regs *)STM32_PWR_BASE)

/******************************************************************************
 * Serial Peripheral Interface
 ******************************************************************************/
struct stm32_spi_regs
{
  	uint16_t cr1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  	uint16_t reserved0;  /*!< Reserved, 0x02                                                           */
  	uint16_t cr2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  	uint16_t reserved1;  /*!< Reserved, 0x06                                                           */
  	uint16_t sr;         /*!< SPI status register,                                Address offset: 0x08 */
  	uint16_t reserved2;  /*!< Reserved, 0x0A                                                           */
  	uint16_t dr;         /*!< SPI data register,                                  Address offset: 0x0C */
  	uint16_t reserved3;  /*!< Reserved, 0x0E                                                           */
  	uint16_t crcpr;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  	uint16_t reserved4;  /*!< Reserved, 0x12                                                           */
  	uint16_t rxcrcr;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  	uint16_t reserved5;  /*!< Reserved, 0x16                                                           */
  	uint16_t txcrcr;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  	uint16_t reserved6;  /*!< Reserved, 0x1A                                                           */
  	uint16_t i2scfgr;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  	uint16_t reserved7;  /*!< Reserved, 0x1E                                                           */
  	uint16_t i2spr;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
  	uint16_t reserved8;  /*!< Reserved, 0x22                                                           */
};

/*
 * Serial Peripheral Interface base
 */
#define STM32_SPI5_BASE       	(STM32_APB2PERIPH_BASE + 0x5000)
#define STM32_SPI6_BASE       	(STM32_APB2PERIPH_BASE + 0x5400)
#define STM32_SPI5				((volatile struct stm32_spi_regs *)STM32_SPI5_BASE)
#define STM32_SPI6				((volatile struct stm32_spi_regs *)STM32_SPI6_BASE)


/******************************************************************************
 * System configuration controller Interface
 ******************************************************************************/

struct stm32_syscfg_regs
{
  uint32_t memrmp;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  uint32_t pmc;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  uint32_t exticr[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t reserved[2];  /*!< Reserved, 0x18-0x1C                                                          */ 
  uint32_t cmpcr;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
};


/*
 * System configuration controller registers base
 */
#define STM32_SYSCFG_BASE 	(STM32_APB2PERIPH_BASE + 0x3800)
#define STM32_SYSCFG		((volatile struct stm32_syscfg_regs *)STM32_SYSCFG_BASE)

#endif /* _MACH_STM32_H_ */
