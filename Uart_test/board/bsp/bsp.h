/***************************************************************************//**
 * @file
 * @brief Board support package API definitions.
 * @author Energy Micro AS
 * @version 1.0.0
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
#ifndef __BSP_H
#define __BSP_H

#include <stdbool.h>
#include "bspconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BSPCOMMON API common for all kits */ /** @{ */

#define BSP_STATUS_OK                 0     /**< BSP API return code, no errors. */
#define BSP_STATUS_ILLEGAL_PARAM      (-1)  /**< BSP API return code, illegal input parameter. */
#define BSP_STATUS_NOT_IMPLEMENTED    (-2)  /**< BSP API return code, function not implemented (dummy). */

/* Initialization flag bitmasks for BSP_Init(). */
#define BSP_INIT_DK_SPI     0x01  /**< Mode flag for @ref BSP_Init(), init DK in SPI mode (DK3x50 only). */
#define BSP_INIT_DK_EBI     0x02  /**< Mode flag for @ref BSP_Init(), init DK in EBI mode (DK3x50 only). */
#define BSP_INIT_STK_BCUART 0x04  /**< Mode flag for @ref BSP_Init(), init board controller UART on STK's. */

/** @} */

#if defined( BSP_DK )
/** @addtogroup BSP_DK API for DK's */ /** @{ */

/** Display Control */
typedef enum
{
  BSP_Display_EBI,          /**< SSD2119 TFT controller driven by EFM32GG EBI interface */
  BSP_Display_SPI,          /**< SSD2119 TFT controller driven by EFM32GG SPI interface */
  BSP_Display_BC,           /**< SSD2119 TFT controller driven by board controller (AEM) */
  BSP_Display_PowerEnable,  /**< SSD2119 Enable power  */
  BSP_Display_PowerDisable, /**< SSD2119 Disable power  */
  BSP_Display_ResetAssert,  /**< Hold SSD2119 in reset */
  BSP_Display_ResetRelease, /**< Release SSD2119 in reset */
  BSP_Display_Mode8080,     /**< Configure SSD2119 for 8080 mode of operation  */
  BSP_Display_ModeGeneric,  /**< Configure SSD2119 for Generic+SPI mode of operation */
} BSP_Display_TypeDef;

/** Bus control access mode */
typedef enum
{
  BSP_BusControl_OFF,    /**< Board control disable */
  BSP_BusControl_DIRECT, /**< GPIO direct drive (n/a) */
  BSP_BusControl_SPI,    /**< Configure Board controller for SPI mode */
  BSP_BusControl_EBI,    /**< Configure Board controller for EBI mode */
} BSP_BusControl_TypeDef;

#if defined( BSP_DK_3200 )                        /* GxxxDK */

/** Peripherals control structure for Gxxx_DK's. */
typedef enum
{
  BSP_ACCEL          = BC_PERCTRL_ACCEL,          /**< Accelerometer */
  BSP_AMBIENT        = BC_PERCTRL_AMBIENT,        /**< Light sensor */
  BSP_POTMETER       = BC_PERCTRL_POTMETER,       /**< Potentiometer */
  BSP_RS232A         = BC_PERCTRL_RS232A,         /**< Serial port A */
  BSP_RS232B         = BC_PERCTRL_RS232B,         /**< Serial port B */
  BSP_SPI            = BC_PERCTRL_SPI,            /**< SPI interface */
  BSP_I2C            = BC_PERCTRL_I2C,            /**< I2C interface */
  BSP_IRDA           = BC_PERCTRL_IRDA,           /**< IrDA interface */
  BSP_ANALOG_SE      = BC_PERCTRL_ANALOG_SE,      /**< Single ended analog input */
  BSP_ANALOG_DIFF    = BC_PERCTRL_ANALOG_DIFF,    /**< Differential analog input */
  BSP_AUDIO_OUT      = BC_PERCTRL_AUDIO_OUT,      /**< Audio Out */
  BSP_AUDIO_IN       = BC_PERCTRL_AUDIO_IN,       /**< Audio In */
  BSP_ACCEL_GSEL     = BC_PERCTRL_ACCEL_GSEL,     /**< Accelerometer range select */
  BSP_ACCEL_SELFTEST = BC_PERCTRL_ACCEL_SELFTEST, /**< Accelerometer selftest mode */
  BSP_RS232_SHUTDOWN = BC_PERCTRL_RS232_SHUTDOWN, /**< Disable RS232 */
  BSP_IRDA_SHUTDOWN  = BC_PERCTRL_IRDA_SHUTDOWN   /**< Disable IrDA */
#ifdef DOXY_DOC_ONLY
} BSP_Peripheral_Typedef;                         /* Hack for doxygen doc ! */
#else
} BSP_Peripheral_TypeDef;
#endif
#endif /* BSP_DK_3200 */

#if defined( BSP_DK_3201 )                        /* DK3750 or DK3550 */

/** Peripherals control structure for DK3750, DK3650 or DK3550. */
typedef enum
{
  BSP_RS232_SHUTDOWN, /**< Disable RS232 */
  BSP_RS232_UART,     /**< UART control of RS232 */
  BSP_RS232_LEUART,   /**< LEUART control of RS232 */
  BSP_I2C,            /**< I2C interface */
  BSP_ETH,            /**< Ethernet */
  BSP_I2S,            /**< Audio I2S */
  BSP_TRACE,          /**< ETM Trace */
  BSP_TOUCH,          /**< Display touch interface */
  BSP_AUDIO_IN,       /**< Audio In */
  BSP_AUDIO_OUT,      /**< Audio Out */
  BSP_ANALOG_DIFF,    /**< Differential analog input */
  BSP_ANALOG_SE,      /**< Single ended analog input */
  BSP_MICROSD,        /**< MicroSD SPI interace */
  BSP_TFT,            /**< SSD2119 TFT controller */
} BSP_Peripheral_TypeDef;
#endif  /* BSP_DK_3201 */

/** @} */
#endif  /* BSP_DK */

/************************** The BSP API *******************************/

int       BSP_Disable(void);
int       BSP_Init(uint32_t flags);
int       BSP_LedClear(int ledNo);
int       BSP_LedGet(int ledNo);
int       BSP_LedSet(int ledNo);
uint32_t  BSP_LedsGet(void);
int       BSP_LedsInit(void);
int       BSP_LedsSet(uint32_t leds);
int       BSP_LedToggle(int ledNo);


#if defined( BSP_DK )
int       BSP_BusControlModeSet(BSP_BusControl_TypeDef mode);
uint32_t  BSP_DipSwitchGet(void);
int       BSP_DisplayControl(BSP_Display_TypeDef option);
int       BSP_EnergyModeSet(uint16_t energyMode);
int       BSP_InterruptDisable(uint16_t flags);
int       BSP_InterruptEnable(uint16_t flags);
int       BSP_InterruptFlagsClear(uint16_t flags);
uint16_t  BSP_InterruptFlagsGet(void);
uint16_t  BSP_JoystickGet(void);
int       BSP_PeripheralAccess(BSP_Peripheral_TypeDef perf, bool enable);
uint16_t  BSP_PushButtonsGet(void);
uint16_t  BSP_RegisterRead(volatile uint16_t *addr);
int       BSP_RegisterWrite(volatile uint16_t *addr, uint16_t data);
#endif

#if defined( BSP_STK )
float     BSP_CurrentGet(void);
int       BSP_EbiInit(void);
float     BSP_VoltageGet(void);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* __BSP_H */
