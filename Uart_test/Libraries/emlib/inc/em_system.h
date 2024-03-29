/***************************************************************************//**
 * @file
 * @brief System API
 * @author Energy Micro AS
 * @version 3.0.3
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
 ******************************************************************************/
#ifndef __EM_SYSTEM_H
#define __EM_SYSTEM_H

#include <stdbool.h>
#include "em_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup EM_Library
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup SYSTEM
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

/** Chip revision details */
typedef struct
{
  uint8_t major; /**< Major revision number */
  uint8_t minor; /**< Minor revision number */
} SYSTEM_ChipRevision_TypeDef;

#if defined(_EFM32_WONDER_FAMILY)
/** Floating point coprocessor access modes. */
typedef enum
{
  fpuAccessDenied         = (0x0 << 20),  /**< Access denied, any attempted access generates a NOCP UsageFault. */
  fpuAccessPrivilegedOnly = (0x5 << 20),  /**< Privileged access only, an unprivileged access generates a NOCP UsageFault. */
  fpuAccessReserved       = (0xA << 20),  /**< Reserved. */
  fpuAccessFull           = (0xF << 20)   /**< Full access. */
} SYSTEM_FpuAccess_TypeDef;
#endif

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void     SYSTEM_ChipRevisionGet(SYSTEM_ChipRevision_TypeDef *rev);
uint32_t SYSTEM_GetCalibrationValue(volatile uint32_t *regAddress);

#if defined(_EFM32_WONDER_FAMILY)
/***************************************************************************//**
 * @brief
 *   Set floating point coprocessor (FPU) access mode.
 *
 * @param[in] accessMode
 *   Floating point coprocessor access mode. See @ref SYSTEM_FpuAccess_TypeDef
 *   for details.
 ******************************************************************************/
__STATIC_INLINE void SYSTEM_FpuAccessModeSet(SYSTEM_FpuAccess_TypeDef accessMode)
{
  SCB->CPACR = (SCB->CPACR & ~(0xF << 20)) | accessMode;
}
#endif

/***************************************************************************//**
 * @brief
 *   Get the unique number for this part.
 *
 * @return
 *   Unique number for this part.
 ******************************************************************************/
__STATIC_INLINE uint64_t SYSTEM_GetUnique(void)
{
  return ((uint64_t) ((uint64_t) DEVINFO->UNIQUEH << 32) | (uint64_t) DEVINFO->UNIQUEL);
}

/***************************************************************************//**
 * @brief
 *   Get the production revision for this part.
 *
 * @return
 *   Production revision for this part.
 ******************************************************************************/
__STATIC_INLINE uint8_t SYSTEM_GetProdRev(void)
{
  return ((DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK)
                         >> _DEVINFO_PART_PROD_REV_SHIFT);
}

/***************************************************************************//**
 * @brief
 *   Get the SRAM size (in KB).
 *
 * @return
 *   The size of the internal SRAM (in KB).
 ******************************************************************************/
__STATIC_INLINE uint16_t SYSTEM_GetSRAMSize(void)
{
#if defined(_EFM32_GECKO_FAMILY)
  /* Early Gecko devices had a bug where SRAM and Flash size were swapped. */
  if (SYSTEM_GetProdRev() < 5)
  {
    return (DEVINFO->MSIZE & _DEVINFO_MSIZE_FLASH_MASK)
                           >> _DEVINFO_MSIZE_FLASH_SHIFT;
  }
#endif
  return (DEVINFO->MSIZE & _DEVINFO_MSIZE_SRAM_MASK)
                         >> _DEVINFO_MSIZE_SRAM_SHIFT;
}

/***************************************************************************//**
 * @brief
 *   Get the flash size (in KB).
 *
 * @return
 *   The size of the internal flash (in KB).
 ******************************************************************************/
__STATIC_INLINE uint16_t SYSTEM_GetFlashSize(void)
{
#if defined(_EFM32_GECKO_FAMILY)
  /* Early Gecko devices had a bug where SRAM and Flash size were swapped. */
  if (SYSTEM_GetProdRev() < 5)
  {
    return (DEVINFO->MSIZE & _DEVINFO_MSIZE_SRAM_MASK)
                           >> _DEVINFO_MSIZE_SRAM_SHIFT;
  }
#endif
  return (DEVINFO->MSIZE & _DEVINFO_MSIZE_FLASH_MASK)
                         >> _DEVINFO_MSIZE_FLASH_SHIFT;
}


/***************************************************************************//**
 * @brief
 *   Get the flash page size in bytes.
 *
 * @return
 *   The page size of the internal flash in bytes.
 ******************************************************************************/
__STATIC_INLINE uint32_t SYSTEM_GetFlashPageSize(void)
{
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  return SYSTEM_GetFlashSize()<512 ? 2048 : 4096;
#else
  /* _EFM32_GECKO_FAMILY || _EFM32_TINY_FAMILY */
  return  512;
#endif
}

/***************************************************************************//**
 * @brief
 *   Get part number of the MCU.
 *
 * @return
 *   The part number of the MCU.
 ******************************************************************************/
__STATIC_INLINE uint16_t SYSTEM_GetPartNumber(void)
{
  return (DEVINFO->PART & _DEVINFO_PART_DEVICE_NUMBER_MASK)
                        >> _DEVINFO_PART_DEVICE_NUMBER_SHIFT;
}

/***************************************************************************//**
 * @brief
 *   Get the calibration temperature (in degrees Celsius).
 *
 * @return
 *   The calibration temperature in Celsius.
 ******************************************************************************/
__STATIC_INLINE uint8_t SYSTEM_GetCalibrationTemperature(void)
{
  return (DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                       >> _DEVINFO_CAL_TEMP_SHIFT;
}

/** @} (end addtogroup SYSTEM) */
/** @} (end addtogroup EM_Library) */

#ifdef __cplusplus
}
#endif

#endif /* __EM_SYSTEM_H */
