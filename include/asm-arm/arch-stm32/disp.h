/*
*********************************************************************************************************
*                                 ENDA PLC GROUP ILI9341 DRIVER MODULE	
*
*                            (c) Copyright 2012; Enda, Inc.; Istanbul, TR
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        ILI9341 DRIVER MODULE
*
*                                             STM32F4XXX
*                                              with the
*                                             HMI Module
*
* Filename      : disp.h
* Version       : V1.00
* Developer(s) 	: tmk
*********************************************************************************************************
*/

#ifndef __DEV_DISP_H__
#define __DEV_DISP_H__

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <stdint.h>


/*
*********************************************************************************************************
*                                           GLOBAL DEFINITIONS
*********************************************************************************************************
*/

#define DISP_XRES			(240)
#define DISP_YRES			(320)
#define DISP_BPP			(16)

#define LCD_FRAME_BUFFER	(0x90700000)
#define LCD_FRAME_BUFFSZ	(0x40000)
#define LCD_FRAME_BUFCNT	(2)


/*******************************************************************************
* Function Name  : Low level driver functions
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int  disp_open	(uint32_t FBAddr);
void disp_close	(void);
void disp_on	(void);
void disp_off	(void);
void disp_fill	(uint32_t FBAddr, uint16_t col);
void disp_setbuf(void *FrameBuf);


/*
*********************************************************************************************************
*                                         LCD LOW LEVEL FUNTIONS
*********************************************************************************************************
*/

#endif //__DEV_DISP_H__
