/*
 * GFA.h
 *
 *  Created on: March 2016
 *      Author: Aaron Miyasaki and Jacob Greig-Prine
 */
//***************************************************************************
//
//  Curent Version: $Revision$
//  Last Revision:  $Date$
//  Revised by:     $User$
//
//  Revision History:   See CVS Log
//
//***************************************************************************/
#include <msp430f2618.h>
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
//#if !defined(HIPS_H__INCLUDED)  // Only include header file once
//#define HIPS_H__INCLUDED

#ifndef GFA_H_
#define GFA_H_

/* definitions */
typedef signed char int8;
typedef unsigned char uint8;
typedef signed short int16;
typedef unsigned short uint16;
typedef signed long int32;
typedef unsigned long uint32;

#define SER_BUFFER_SIZE  64

#define SYSCLK   7500000        // SYSCLK / SPI_RATE should be evenly divisible
#define SPI_RATE 750000
#define FPGACLK  50000000

#define ISRFLG_TWOHZ    BIT2
#define ISRFLG_TIC_BIT  BIT4
#define ISRFLG_USB_BIT  BIT6

#define ON      1
#define OFF     0
#define TRUE    1
#define FALSE   0
#define ERR_OK  0
#define CR      0x0d
#define BS      0x08

#define ERR_VALUE -1
#define DELAY_CONST 12
#define CODEC_RATE_ADDR 1
#define NUM_FREQ  15 // Number sample rate frequencies

#define CS_LOW()  P3OUT &= ~BIT0        // Card Select
#define CS_HIGH() P3OUT |= BIT0         // Card Deselect

#define SD_FPGA_LOW()  P4OUT &= ~BIT1        // SD / FPGA Select
#define SD_FPGA_HIGH() P4OUT |= BIT1         // SD / FPGA Deselect

#define sEn_Latch() P3OUT &= ~BIT4      // FPGA SR Control
#define sEn_Shift() P3OUT |= BIT4       // FPGA SR Control

#define RECORD()        P1IN & BIT2     // FPGA SR Control
#define SD_PRESENT()    P1IN & BIT4         // CP
#define SD_WRITE_PROTECT()  P1IN & BIT3     // WP

/* File function return code (FRESULT) */
typedef enum {
    FR_OK = 0,          /* 0 */
    FR_DISK_ERR,        /* 1 */
    FR_INT_ERR,         /* 2 */
    FR_NOT_READY,       /* 3 */
    FR_NO_FILE,         /* 4 */
    FR_NO_PATH,         /* 5 */
    FR_INVALID_NAME,    /* 6 */
    FR_DENIED,          /* 7 */
    FR_EXIST,           /* 8 */
    FR_INVALID_OBJECT,  /* 9 */
    FR_WRITE_PROTECTED, /* 10 */
    FR_INVALID_DRIVE,   /* 11 */
    FR_NOT_ENABLED,     /* 12 */
    FR_NO_FILESYSTEM,   /* 13 */
    FR_MKFS_ABORTED,    /* 14 */
    FR_TIMEOUT,         /* 15 */
    FR_RW_ERROR         // 16
} FRESULT;

typedef struct filePoint_t filePoint_t;

struct filePoint_t
{
    char menuTitle[16];  // Title of menu to display.
    char subMenu[16];   // Sub-menu to invoke if menu is selected.
};

#define ERROR_STR_NUMBER    17
#define FILE_LIMIT          64
#define Num_of_Results      500

//void InitUart(void);
//void InitSystem(void);
//void InitTimers(void);
//void Set_DCO(void);

//void freqCalc(volatile unsigned int voltage[Num_of_Results], int *offset, double frequency[Num_of_Results]);
//double voltConv(volatile unsigned int voltage[Num_of_Results]);

#endif //  if !defined(MSPUART1LIB_H__INCLUDED)


// #endif /* GFA_H_ */
