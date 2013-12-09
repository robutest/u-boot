/*
 * (C) Copyright 2011
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
#include <lcd.h>
#include <ili9341.h>

#ifdef CONFIG_LCD_ILI9341

#define XRES			DISP_XRES
#define YRES			DISP_YRES
#define BIT_PER_PIXEL	DISP_BPP

vidinfo_t panel_info = {
	vl_col:		XRES,
	vl_row:		YRES,
	vl_bpix:	4,
};

void *lcd_base;						/* Start of framebuffer memory */
void *lcd_console_address;			/* Start of console buffer     */

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

short console_col;
short console_row;


void lcd_enable(void)
{
	/* Display On */
}

void lcd_disable(void)
{
	/* Display Off */
}

ulong calc_fbsize(void)
{
#ifdef CONFIG_LCD_ILI9341_DOUBLE_BUFFER
	int buffers_cnt = 2;
#else
	int buffers_cnt = 1;
#endif
	return ((panel_info.vl_col * panel_info.vl_row *
		NBITS(panel_info.vl_bpix)) / 8) * buffers_cnt;
}

/*
 * if overwrite_console returns 1, the stdin, stderr and stdout
 * are switched to the serial port, else the settings in the
 * environment are used
 */

int overwrite_console(void)
{
	/* we want to stdout / stderr on LCD */
	return 0;
}


void lcd_ctrl_init(void *lcdbase)
{
	u32 mem_len = XRES * YRES * BIT_PER_PIXEL / 8;

	if (!lcdbase)
		return;

	memset(lcdbase, 0, mem_len);
}

#endif /* CONFIG_LCD_ILI9341 */
