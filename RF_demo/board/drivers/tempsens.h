/***************************************************************************//**
 * @file
 * @brief Temperature sensor driver for DS75 temperature sensor compatible
 *   device on the DVK.
 * @author Energy Micro AS
 * @version 2.0.1
 *******************************************************************************
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

#ifndef __TEMPSENS_H
#define __TEMPSENS_H

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

#include "efm32.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C device address for temperature sensor on DVK */
#define TEMPSENS_DVK_ADDR    0x90


/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/

/** Available registers in DS75 sensor device */
typedef enum
{
  tempsensRegTemp       = 0,   /**< Temperature register (read-only) */
  tempsensRegConfig     = 1,   /**< Configuration register */
  tempsensRegHysteresis = 2,   /**< Hysteresis register */
  tempsensRegShutdown   = 3    /**< Overtemperature shutdown register */
} TEMPSENS_Register_TypeDef;


/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

/** Structure used to fetch temperature using integer numbers. */
typedef struct
{
  /** Integer part of temperature, including sign */
  int16_t i;

  /**
   * Fractional part of temperature in 1/10000 parts, including sign. Ie 0.5
   * will be stored as 5000 and -0.5 as -5000.
   */
  int16_t f;
} TEMPSENS_Temp_TypeDef;


/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void TEMPSENS_Celsius2Fahrenheit(TEMPSENS_Temp_TypeDef *temp);
int TEMPSENS_RegisterGet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t *val);
int TEMPSENS_RegisterSet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t val);
int TEMPSENS_TemperatureGet(I2C_TypeDef *i2c,
                            uint8_t addr,
                            TEMPSENS_Temp_TypeDef *temp);

#ifdef __cplusplus
}
#endif

/** @} (end group Drivers) */

#endif /* __TEMPSENS_H */
