/*
 * (C) Copyright 2011, 2012
 *
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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
 * Board specific code for the STMicroelectronic STM3220G-EVAL board
 */

#include <common.h>
#include <asm/arch/fmc.h>
#include <asm/arch/disp.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Early hardware init.
 */
int board_init(void)
{
	/*
	 * Initialize the SDRAM controller
	 */
	fmc_dram_init();

#if defined(CONFIG_LCD)
	/*
	 * Open the lcd driver
	 */
	disp_open(CONFIG_FB_ADDR);

	/*
	 * Initialize the U-boot variables
	 */
	gd->fb_base = CONFIG_FB_ADDR;
#endif

	return 0;
}

/*
 * Dump pertinent info to the console.
 */
int checkboard(void)
{
	printf("Board: STM32F429I-DISCOVERY board,%s\n",
		CONFIG_SYS_BOARD_REV_STR);

	return 0;
}

/*
 * Setup external RAM.
 */
int dram_init(void)
{
	/*
	 * Fill in global info with description of SRAM configuration
	 */
	gd->bd->bi_dram[0].start = CONFIG_SYS_RAM_BASE;
	gd->bd->bi_dram[0].size  = CONFIG_SYS_RAM_SIZE;

	return 0;
}

#ifdef CONFIG_STM32_ETH
/*
 * Register ethernet driver
 */
int board_eth_init(bd_t *bis)
{
	return stm32_eth_init(bis);
}
#endif

