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
 * Configuration settings for the STMicroelectronic STM3220G-EVAL board.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * Disable debug messages
 */
#undef DEBUG

/*
 * This is an ARM Cortex-M4 CPU core. Also use the common Cortex-M3 code.
 */
#define CONFIG_SYS_ARMCORTEXM3
#define CONFIG_SYS_ARMCORTEXM4

/*
 * This is the STM32-F4 device
 */
#define CONFIG_SYS_STM32
#define CONFIG_SYS_STM32F43X

/*
 * Enable GPIO driver
 */
#define CONFIG_STM32F2_GPIO

/*
 * Display CPU and Board information
 */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1

#define CONFIG_SYS_BOARD_REV_STR	"Rev 1.0"

/*
 * Monitor prompt
 */
#define CONFIG_SYS_PROMPT			"STM32429-DISCO> "

/*
 * We want to call the CPU specific initialization
 */
#define CONFIG_ARCH_CPU_INIT

/*
 * Clock configuration (see mach-stm32/clock.c for details):
 * - use PLL as the system clock;
 * - use HSE as the PLL source;
 * - configure PLL to get 180MHz system clock.
 */
#define CONFIG_STM32_SYS_CLK_PLL
#define CONFIG_STM32_PLL_SRC_HSE
#define CONFIG_STM32_HSE_HZ			(8000000)		/* 8 MHz */
#define CONFIG_STM32_PLL_M			(8)
#define CONFIG_STM32_PLL_N			(360)
#define CONFIG_STM32_PLL_P			(2)
#define CONFIG_STM32_PLL_Q			(7)

/*
 * Number of clock ticks in 1 sec
 */
#define CONFIG_SYS_HZ				1000

/*
 * Enable/disable h/w watchdog
 */
#undef CONFIG_HW_WATCHDOG

/*
 * No interrupts
 */
#undef CONFIG_USE_IRQ

/*
 * Memory layout configuration
 */
#define CONFIG_MEM_NVM_BASE			(0x08000000)
#define CONFIG_MEM_NVM_LEN			(2 * 1024 * 1024)

#define CONFIG_MEM_RAM_BASE			(0x20000000)
#define CONFIG_MEM_RAM_LEN			(20 * 1024)
#define CONFIG_MEM_RAM_BUF_LEN		(88 * 1024)
#define CONFIG_MEM_MALLOC_LEN		(64 * 1024)
#define CONFIG_MEM_STACK_LEN		( 4 * 1024)

/*
 * malloc() pool size
 */

/*
 * Use 1 MB at the end of the external memory for the malloc() pool
 */
//#define CONFIG_SYS_MALLOC_EXT_BASE	(0x90600000)
//#define CONFIG_SYS_MALLOC_EXT_LEN	(1024 * 1024)

/*
 * The generic code still needs CONFIG_SYS_MALLOC_LEN to calculate the base
 * address of the global data (`gd`) structure.
 */
#define CONFIG_SYS_MALLOC_LEN		CONFIG_MEM_MALLOC_LEN


/*
 * Configuration of the external PSRAM memory
 */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_RAM_SIZE			(8 * 1024 * 1024)

#define CONFIG_SYS_RAM_BASE			(0x90000000)

#define CONFIG_LCD
#ifdef  CONFIG_LCD
#define LCD_BPP						LCD_COLOR16
#define CONFIG_BMP_16BPP			1
#define CONFIG_SPLASH_SCREEN		1
#define CONFIG_CMD_BMP
#define CONFIG_FB_ADDR				(0x90700000)
#define CONFIG_LCD_ILI9341
#define CONFIG_LCD_ILI9341_DOUBLE_BUFFER
#define CONFIG_SYS_WHITE_ON_BLACK
#endif

/*
 * Configuration of the external Flash memory
 */
#define CONFIG_SYS_NO_FLASH

/*
 * Store env in memory only
 */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE				(1 * 1024)

/*
 * Serial console configuration
 */
#define CONFIG_STM32_USART_CONSOLE
#define CONFIG_STM32_USART_PORT			3	/* USART3 */
#define CONFIG_STM32_USART_TX_IO_PORT	2	/* PORTC */
#define CONFIG_STM32_USART_RX_IO_PORT	2	/* PORTC */
#define CONFIG_STM32_USART_TX_IO_PIN	10	/* GPIO10 */
#define CONFIG_STM32_USART_RX_IO_PIN	11	/* GPIO11 */

#define CONFIG_BAUDRATE					115200
#define CONFIG_SYS_BAUDRATE_TABLE		{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Ethernet configuration
 */
#undef  CONFIG_NET_MULTI
#undef  CONFIG_STM32_ETH

/*
 * Ethernet RX buffers are malloced from the internal SRAM (more precisely,
 * from CONFIG_SYS_MALLOC_LEN part of it). Each RX buffer has size of 1536B.
 * So, keep this in mind when changing the value of the following config,
 * which determines the number of ethernet RX buffers (number of frames which
 * may be received without processing until overflow happens).
 */
#define CONFIG_SYS_RX_ETH_BUFFER	4

/*
 * Console I/O buffer size
 */
#define CONFIG_SYS_CBSIZE			256

/*
 * Print buffer size
 */
#define CONFIG_SYS_PBSIZE      		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MEMTEST_START	(CONFIG_SYS_RAM_BASE)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_RAM_BASE + CONFIG_SYS_RAM_SIZE)

/*
 * Needed by "loadb"
 */
#define CONFIG_SYS_LOAD_ADDR		CONFIG_SYS_RAM_BASE

/*
 * Monitor is actually in eNVM. In terms of U-Boot, it is neither "flash",
 * not RAM, but CONFIG_SYS_MONITOR_BASE must be defined.
 */
#define CONFIG_SYS_MONITOR_BASE		0x0

/*
 * Monitor is not in flash. Needs to define this to prevent
 * U-Boot from running flash_protect() on the monitor code.
 */
#define CONFIG_MONITOR_IS_IN_RAM	1

/*
 * Enable all those monitor commands that are needed
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_BOOTD
#undef CONFIG_CMD_CONSOLE
#undef CONFIG_CMD_ECHO
#undef CONFIG_CMD_EDITENV
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_XIMG

/*
 * To save memory disable long help
 */
#undef CONFIG_SYS_LONGHELP

/*
 * Max number of command args
 */
#define CONFIG_SYS_MAXARGS		16

/*
 * Auto-boot sequence configuration
 */
#define CONFIG_BOOTDELAY		0
#define CONFIG_HOSTNAME			stm32429-disco
#define CONFIG_BOOTARGS			"stm32_platform=stm32429-disco mem=7M "\
								"console=ttyS2,115200n8 consoleblank=0 "\
								"root=/dev/mtdblock0 rdinit=/sbin/init "\
								"video=vfb:enable,fbmem:0x90700000,fbsize:0x100000"
#define CONFIG_BOOTCOMMAND		"run flashboot"

#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_INFO_QUIET

/*
 * Short-cuts to some useful commands (macros)
 */
#define CONFIG_EXTRA_ENV_SETTINGS				\
	"loadaddr=0x90000000\0"						\
	"addip=setenv bootargs ${bootargs}\0"		\
	"flashaddr=08020000\0"						\
	"flashboot=run addip;bootm ${flashaddr}\0"	\
	"image=uImage\0"							\
	"stdin=serial\0"							\
	"stdout=serial\0"							\
	"stderr=serial\0"

/*
 * Linux kernel boot parameters configuration
 */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG

#endif /* __CONFIG_H */
