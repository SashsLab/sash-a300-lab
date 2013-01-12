/**************************************************************************//**
 * @file
 * @brief Segment LCD Font And Layout for the EFM32TG_STK3300 starter kit
 * @author Energy Micro AS
 * @version 1.0.1
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
#include "segmentfont.h"

/**************************************************************************//**
 * @brief Working instance of LCD display
 *****************************************************************************/
const MCU_DISPLAY EFM_Display = {
  .Text        = {
    { /* 1 */
      .com[0] = 1, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 13, .bit[1] = 14, .bit[2] = 14, .bit[3] = 14,

      .com[4] = 7, .com[5] = 3, .com[6] = 4, .com[7] = 2,
      .bit[4] = 13, .bit[5] = 13, .bit[6] = 13, .bit[7] = 13,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 14, .bit[9] = 14, .bit[10] = 14, .bit[11] = 14,

      .com[12] = 5, .com[13] = 6,
      .bit[12] = 13, .bit[13] = 13
    },
    { /* 2 */
      .com[0] = 1, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 15, .bit[1] = 16, .bit[2] = 16, .bit[3] = 16,

      .com[4] = 7, .com[5] = 3, .com[6] = 4, .com[7] = 2,
      .bit[4] = 15, .bit[5] = 15, .bit[6] = 15, .bit[7] = 15,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 16, .bit[9] = 16, .bit[10] = 16, .bit[11] = 16,

      .com[12] = 5, .com[13] = 6,
      .bit[12] = 15, .bit[13] = 15
    },
    { /* 3 */
      .com[0] = 1, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 17, .bit[1] = 18, .bit[2] = 18, .bit[3] = 18,

      .com[4] = 7, .com[5] = 3, .com[6] = 4, .com[7] = 2,
      .bit[4] = 17, .bit[5] = 17, .bit[6] = 17, .bit[7] = 17,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 18, .bit[9] = 18, .bit[10] = 18, .bit[11] = 18,

      .com[12] = 5, .com[13] = 6,
      .bit[12] = 17, .bit[13] = 17
    },
    { /* 4 */
      .com[0] = 1, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 19, .bit[1] = 28, .bit[2] = 28, .bit[3] = 28,

      .com[4] = 7, .com[5] = 3, .com[6] = 4, .com[7] = 2,
      .bit[4] = 19, .bit[5] = 19, .bit[6] = 19, .bit[7] = 19,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 28, .bit[9] = 28, .bit[10] = 28, .bit[11] = 28,

      .com[12] = 5, .com[13] = 6,
      .bit[12] = 19, .bit[13] = 19
    },
    { /* 5 */
      .com[0] = 0, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 29, .bit[1] = 30, .bit[2] = 30, .bit[3] = 30,

      .com[4] = 6, .com[5] = 2, .com[6] = 3, .com[7] = 1,
      .bit[4] = 29, .bit[5] = 29, .bit[6] = 29, .bit[7] = 29,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 30, .bit[9] = 30, .bit[10] = 30, .bit[11] = 30,

      .com[12] = 4, .com[13] = 5,
      .bit[12] = 29, .bit[13] = 29
    },
    { /* 6 */
      .com[0] = 0, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 31, .bit[1] = 32, .bit[2] = 32, .bit[3] = 32,

      .com[4] = 6, .com[5] = 2, .com[6] = 3, .com[7] = 1,
      .bit[4] = 31, .bit[5] = 31, .bit[6] = 31, .bit[7] = 31,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 32, .bit[9] = 32, .bit[10] = 32, .bit[11] = 32,

      .com[12] = 4, .com[13] = 5,
      .bit[12] = 31, .bit[13] = 31
    },
    { /* 7 */
      .com[0] = 1, .com[1] = 1, .com[2] = 5, .com[3] = 7,
      .bit[0] = 33, .bit[1] = 34, .bit[2] = 34, .bit[3] = 34,

      .com[4] = 7, .com[5] = 3, .com[6] = 4, .com[7] = 2,
      .bit[4] = 33, .bit[5] = 33, .bit[6] = 33, .bit[7] = 33,

      .com[8] = 3, .com[9] = 2, .com[10] = 4, .com[11] = 6,
      .bit[8] = 34, .bit[9] = 34, .bit[10] = 34, .bit[11] = 34,

      .com[12] = 5, .com[13] = 6,
      .bit[12] = 33, .bit[13] = 33
    },
  },
  .Number      = {
    {
      .com[0] = 7, .com[1] = 5, .com[2] = 2, .com[3] = 1,
      .bit[0] = 35, .bit[1] = 35, .bit[2] = 35, .bit[3] = 35,

      .com[4] = 3, .com[5] = 6, .com[6] = 4,
      .bit[4] = 35, .bit[5] = 35, .bit[6] = 35,
    },
    {
      .com[0] = 7, .com[1] = 5, .com[2] = 2, .com[3] = 1,
      .bit[0] = 36, .bit[1] = 36, .bit[2] = 36, .bit[3] = 36,

      .com[4] = 3, .com[5] = 6, .com[6] = 4,
      .bit[4] = 36, .bit[5] = 36, .bit[6] = 36,
    },
    {
      .com[0] = 7, .com[1] = 5, .com[2] = 2, .com[3] = 1,
      .bit[0] = 37, .bit[1] = 37, .bit[2] = 37, .bit[3] = 37,

      .com[4] = 3, .com[5] = 6, .com[6] = 4,
      .bit[4] = 37, .bit[5] = 37, .bit[6] = 37,
    },
    {
      .com[0] = 7, .com[1] = 5, .com[2] = 2, .com[3] = 1,
      .bit[0] = 38, .bit[1] = 38, .bit[2] = 38, .bit[3] = 38,

      .com[4] = 3, .com[5] = 6, .com[6] = 4,
      .bit[4] = 38, .bit[5] = 38, .bit[6] = 38,
    },
  },
  .EMode       = {
    .com[0] = 0, .bit[0] = 39,
    .com[1] = 1, .bit[1] = 39,
    .com[2] = 7, .bit[2] = 39,
    .com[3] = 2, .bit[3] = 39,
    .com[4] = 6, .bit[4] = 39,
  },
  .ARing       = {
    .com[0] = 0, .bit[0] = 19,
    .com[1] = 0, .bit[1] = 18,
    .com[2] = 0, .bit[2] = 17,
    .com[3] = 0, .bit[3] = 16,

    .com[4] = 0, .bit[4] = 15,
    .com[5] = 0, .bit[5] = 14,
    .com[6] = 0, .bit[6] = 13,
    .com[7] = 0, .bit[7] = 12,
  },
  .Battery     = {
    .com[0] = 0, .bit[0] = 33,
    .com[1] = 0, .bit[1] = 37,
    .com[2] = 0, .bit[2] = 36,
    .com[3] = 0, .bit[3] = 38,
  }
};

/**************************************************************************//**
 * @brief
 * Defines higlighted segments for the alphabet, starting from "blank" (SPACE)
 * Uses bit pattern as defined for text segments above.
 * E.g. a capital O, would have bits 0 1 2 3 4 5 => 0x003f defined
 *****************************************************************************/
const uint16_t EFM_Alphabet[] = {
  0x0000, /* space */
  0x1100, /* ! */
  0x0280, /* " */
  0x0000, /* # */
  0x0000, /* $ */
  0x0602, /* % */
  0x0000, /* & */
  0x0020, /* ' */
  0x0039, /* ( */
  0x000f, /* ) */
  0x0000, /* * */
  0x1540, /* + */
  0x2000, /* , */
  0x0440, /* - */
  0x1000, /* . */
  0x2200, /* / */

  0x003f, /* 0 */
  0x0006, /* 1 */
  0x045b, /* 2 */
  0x044f, /* 3 */
  0x0466, /* 4 */
  0x046d, /* 5 */
  0x047d, /* 6 */
  0x0007, /* 7 */
  0x047f, /* 8 */
  0x046f, /* 9 */

  0x0000, /* : */
  0x0000, /* ; */
  0x0a00, /* < */
  0x0000, /* = */
  0x2080, /* > */
  0x0000, /* ? */
  0xffff, /* @ */

  0x0477, /* A */
  0x0a79, /* B */
  0x0039, /* C */
  0x20b0, /* D */
  0x0079, /* E */
  0x0071, /* F */
  0x047d, /* G */
  0x0476, /* H */
  0x0006, /* I */
  0x000e, /* J */
  0x0a70, /* K */
  0x0038, /* L */
  0x02b6, /* M */
  0x08b6, /* N */
  0x003f, /* O */
  0x0473, /* P */
  0x083f, /* Q */
  0x0c73, /* R */
  0x046d, /* S */
  0x1101, /* T */
  0x003e, /* U */
  0x2230, /* V */
  0x2836, /* W */
  0x2a80, /* X */
  0x046e, /* Y */
  0x2209, /* Z */

  0x0039, /* [ */
  0x0880, /* backslash */
  0x000f, /* ] */
  0x0001, /* ^ */
  0x0008, /* _ */
  0x0100, /* ` */

  0x1058, /* a */
  0x047c, /* b */
  0x0058, /* c */
  0x045e, /* d */
  0x2058, /* e */
  0x0471, /* f */
  0x0c0c, /* g */
  0x0474, /* h */
  0x0004, /* i */
  0x000e, /* j */
  0x0c70, /* k */
  0x0038, /* l */
  0x1454, /* m */
  0x0454, /* n */
  0x045c, /* o */
  0x0473, /* p */
  0x0467, /* q */
  0x0450, /* r */
  0x0c08, /* s */
  0x0078, /* t */
  0x001c, /* u */
  0x2010, /* v */
  0x2814, /* w */
  0x2a80, /* x */
  0x080c, /* y */
  0x2048, /* z */

  0x0000,
};

/**************************************************************************//**
 * @brief
 * Defines higlighted segments for the numeric display
 *****************************************************************************/
const uint16_t EFM_Numbers[] =
{
  0x003f, /* 0 */
  0x0006, /* 1 */
  0x005b, /* 2 */
  0x004f, /* 3 */
  0x0066, /* 4 */
  0x006d, /* 5 */
  0x007d, /* 6 */
  0x0007, /* 7 */
  0x007f, /* 8 */
  0x006f, /* 9 */
  0x0077, /* A */
  0x007c, /* b */
  0x0027, /* C */
  0x005e, /* d */
  0x0079, /* E */
  0x0071, /* F */
  0x0040  /* - */
};
