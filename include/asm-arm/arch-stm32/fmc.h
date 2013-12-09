/*
 * (C) Copyright 2011
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _MACH_FMC_H_
#define _MACH_FMC_H_


/**
  * FMC SDRAM Bank address
  */   
#define FMC_SDRAM_BANK_ADDR    	((uint32_t)0xD0000000)

/**
  * FMC SDRAM Size
  */   
#define FMC_SDRAM_BANK_SIZE		((uint32_t)0x00800000)

/**
 * Initialize the specified SDRAM controller
 * @returns             0 on success, < 0 on failure
 */
int fmc_dram_init(void);

#endif /* _MACH_FMC_H_ */
