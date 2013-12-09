/*
 * (C) Copyright 2010,2011
 * Dmitry Konyshev, Emcraft Systems, probables@emcraft.com
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/stm32.h>
#include <asm/arch/fmc.h>
#include <asm/arch/stm32f2_gpio.h>


/*
*********************************************************************************************************
*                                             EXTERNAL FUNCTIONS
*********************************************************************************************************
*/

void udelay(unsigned long usec);


/*
 * External SDRAM GPIOs for FMC:
 *
 +-------------------+--------------------+--------------------+--------------------+
 +                       SDRAM pins assignment                                      +
 +-------------------+--------------------+--------------------+--------------------+
 | PD0  <-> FMC_D2   | PE0  <-> FMC_NBL0  | PF0  <-> FMC_A0    | PG0  <-> FMC_A10   |
 | PD1  <-> FMC_D3   | PE1  <-> FMC_NBL1  | PF1  <-> FMC_A1    | PG1  <-> FMC_A11   |
 | PD8  <-> FMC_D13  | PE7  <-> FMC_D4    | PF2  <-> FMC_A2    | PG8  <-> FMC_SDCLK |
 | PD9  <-> FMC_D14  | PE8  <-> FMC_D5    | PF3  <-> FMC_A3    | PG15 <-> FMC_NCAS  |
 | PD10 <-> FMC_D15  | PE9  <-> FMC_D6    | PF4  <-> FMC_A4    |--------------------+ 
 | PD14 <-> FMC_D0   | PE10 <-> FMC_D7    | PF5  <-> FMC_A5    |   
 | PD15 <-> FMC_D1   | PE11 <-> FMC_D8    | PF11 <-> FMC_NRAS  | 
 +-------------------| PE12 <-> FMC_D9    | PF12 <-> FMC_A6    | 
                     | PE13 <-> FMC_D10   | PF13 <-> FMC_A7    |    
                     | PE14 <-> FMC_D11   | PF14 <-> FMC_A8    |
                     | PE15 <-> FMC_D12   | PF15 <-> FMC_A9    |
 +-------------------+--------------------+--------------------+
 | PB5 <-> FMC_SDCKE1| 
 | PB6 <-> FMC_SDNE1 | 
 | PC0 <-> FMC_SDNWE |
 +-------------------+  
 */
static const struct stm32f2_gpio_dsc ext_ram_fmc_gpio[] = {
	
	{ STM32F2_GPIO_PORT_B, STM32F2_GPIO_PIN_5,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_B, STM32F2_GPIO_PIN_6,  STM32F2_GPIO_ROLE_FSMC },

	{ STM32F2_GPIO_PORT_C, STM32F2_GPIO_PIN_0,  STM32F2_GPIO_ROLE_FSMC },

	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_0,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_1,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_8,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_9,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_10, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_14, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_D, STM32F2_GPIO_PIN_15, STM32F2_GPIO_ROLE_FSMC },

	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_0,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_1,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_7,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_8,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_9,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_10, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_11, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_12, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_13, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_14, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_E, STM32F2_GPIO_PIN_15, STM32F2_GPIO_ROLE_FSMC },

	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_0,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_1,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_2,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_3,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_4,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_5,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_11, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_12, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_13, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_14, STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_F, STM32F2_GPIO_PIN_15, STM32F2_GPIO_ROLE_FSMC },

	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_0,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_1,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_4,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_5,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_8,  STM32F2_GPIO_ROLE_FSMC },
	{ STM32F2_GPIO_PORT_G, STM32F2_GPIO_PIN_15, STM32F2_GPIO_ROLE_FSMC },
};


/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f429_439xx.s before jump to main.
  *         This function configures the external SDRAM mounted on STM32F429I DISCO board
  *         This SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
#define WAIT_FOR_BUSY(tmo)	do { 	uint32_t timeout = tmo;	\
									uint32_t tmpreg  = STM32_FMC_DRAM->sdsr & 0x00000020;	\
									while ((tmpreg != 0) && (timeout-- > 0))	\
										tmpreg = STM32_FMC_DRAM->sdsr & 0x00000020; } while (0)
static void ExtMemInit(void)
{
	// FMC Configuration
	// Enable the FMC interface clock
	STM32_RCC->ahb3enr |= 0x00000001;

	// SDRAM bank control register configuration
	STM32_FMC_DRAM->sdcr[0] = 0x00001800;
	STM32_FMC_DRAM->sdcr[1] = 0x000019D4;

	// SDRAM bank timing register configuration
	STM32_FMC_DRAM->sdtr[0] = 0x00106000;
	STM32_FMC_DRAM->sdtr[1] = 0x00010361;

	// Clock configuration
	WAIT_FOR_BUSY(0xFFFF);
	STM32_FMC_DRAM->sdcmr = 0x00000009;

	// Delay
	udelay(10000);

	// PALL configuration
	WAIT_FOR_BUSY(0xFFFF);
	STM32_FMC_DRAM->sdcmr = 0x0000000A;

	// Auto refresh configuration
	WAIT_FOR_BUSY(0xFFFF);
	STM32_FMC_DRAM->sdcmr = 0x000000EB;

	// program external memory mode
	WAIT_FOR_BUSY(0xFFFF);
	STM32_FMC_DRAM->sdcmr = 0x0004600C;

	// Set refresh rate
	STM32_FMC_DRAM->sdrtr = (1386 << 1);
	WAIT_FOR_BUSY(0xFFFF);
}//ExtMemInit


int fmc_dram_init(void)
{
	static int common_init_done = 0;
	uint32_t *ptr, len;
	int rv = 0;

	if (!common_init_done) {
		int	i;

		/*
		 * Connect GPIOs to FMC controller
		 */
		for (i = 0; i < ARRAY_SIZE(ext_ram_fmc_gpio); i++) {
			rv = stm32f2_gpio_config(&ext_ram_fmc_gpio[i]);
			if (rv != 0)
				return rv;
		}

		/*
		 * Initialize FMC controller
		 */
		ExtMemInit();

		/*
		 * Clear all memories
		 */
		ptr = (uint32_t*)FMC_SDRAM_BANK_ADDR;
		len = FMC_SDRAM_BANK_SIZE/4;
		while (len--)
			*ptr++ = 0;

		/*
		 * Remapping Bank6 -> Bank4
		 */
		STM32_RCC->apb2enr  |= ((uint32_t)0x00004000);
		STM32_SYSCFG->memrmp = ((uint32_t)0x00000400);

		common_init_done = 1;
	}

	return 0;
}
