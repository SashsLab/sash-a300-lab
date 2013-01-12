/**************************************************************************//**
 * @file
 * @brief Micro SD card driver for the EFM32GG_DK3750 development kit.
 * @author Energy Micro AS
 * @version 2.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#ifndef __MICROSD_H
#define __MICROSD_H

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "efm32.h"
#include "em_gpio.h"
#include "integer.h"

/* Definitions for MMC/SDC command */
#define CMD0      (0)                                                                        /* GO_IDLE_STATE */
#define CMD1      (1)                                                                        /* SEND_OP_COND */
#define ACMD41    (41 | 0x80)                                                                /* SEND_OP_COND (SDC) */
#define CMD8      (8)                                                                        /* SEND_IF_COND */
#define CMD9      (9)                                                                        /* SEND_CSD */
#define CMD10     (10)                                                                       /* SEND_CID */
#define CMD12     (12)                                                                       /* STOP_TRANSMISSION */
#define ACMD13    (13 | 0x80)                                                                /* SD_STATUS (SDC) */
#define CMD16     (16)                                                                       /* SET_BLOCKLEN */
#define CMD17     (17)                                                                       /* READ_SINGLE_BLOCK */
#define CMD18     (18)                                                                       /* READ_MULTIPLE_BLOCK */
#define CMD23     (23)                                                                       /* SET_BLOCK_COUNT */
#define ACMD23    (23 | 0x80)                                                                /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24     (24)                                                                       /* WRITE_BLOCK */
#define CMD25     (25)                                                                       /* WRITE_MULTIPLE_BLOCK */
#define CMD41     (41)                                                                       /* SEND_OP_COND (ACMD) */
#define CMD55     (55)                                                                       /* APP_CMD */
#define CMD58     (58)                                                                       /* READ_OCR */

#define FCLK_SLOW()    (USART0->CLKDIV = 256 * (48000000 / (2 * 100000) - 1));               /* 100K@48MHz */
#define FCLK_FAST()    (USART0->CLKDIV = 256 * (48000000 / (2 * 7000000) - 1));              /* 7MHz@48MHz */

#define CS_LOW()       GPIO->P[4].DOUTCLR = 0x10;
#define CS_HIGH()      GPIO->P[4].DOUTSET = 0x10;

void MICROSD_init(void);

uint8_t xfer_spi(uint8_t data);
BYTE wait_ready(void);
void deselect(void);
int select(void);
void power_on(void);
void power_off(void);
int rcvr_datablock(BYTE *buff, UINT btr);
int xmit_datablock(const BYTE *buff, BYTE token);
BYTE send_cmd(BYTE cmd, DWORD arg);
void MICROSD_deinit(void);

#ifdef __cplusplus
}
#endif

/** @} (end group Drivers) */

#endif
