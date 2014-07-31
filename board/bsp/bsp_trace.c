/**************************************************************************//**
 * @file
 * @brief API for enabling SWO or ETM trace on DK3750 board
 * @author Energy Micro AS
 * @version 1.0.0
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
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "bsp_trace.h"
#include "bsp.h"

#if defined(BSP_ETM_TRACE) && defined( ETM_PRESENT )
/**************************************************************************//**
 * @brief Configure EFM32GG990F1024 for DK3750 ETM trace output
 * @note You need to configure ETM trace on on kit config menu as well!
 *****************************************************************************/
void BSP_TraceEtmSetup(void)
{
  /* Enable peripheral clocks */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
  CMU->HFPERCLKEN0  |= CMU_HFPERCLKEN0_GPIO;
  CMU->OSCENCMD      = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until AUXHFRCO clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY)) ;

  /* Enable Port D, pins 3,4,5,6 for ETM Trace Data output */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_PUSHPULL;
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_PUSHPULL;

  /* Enable Port D, pin 7 for DBG_TCLK */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK) | GPIO_P_MODEL_MODE7_PUSHPULL;

  /* Configure trace output for alternate location */
  GPIO->ROUTE = GPIO->ROUTE | GPIO_ROUTE_TCLKPEN | GPIO_ROUTE_TD0PEN | GPIO_ROUTE_TD1PEN
                | GPIO_ROUTE_TD2PEN | GPIO_ROUTE_TD3PEN
                | GPIO_ROUTE_ETMLOCATION_LOC0;
}
#endif

/**************************************************************************//**
 * @brief Configure trace output for energyAware Profiler
 * @note  Enabling trace will add a 80uA current for the EFM32_Gxxx_STK.
 *        DK's needs to be initialized with SPI-mode:
 * @verbatim BSP_Init(BSP_Init_SPI); @endverbatim
 *****************************************************************************/
void BSP_TraceSwoSetup(void)
{
  /* Debug logic registers */
  volatile uint32_t *dwt_ctrl       = (uint32_t *) 0xE0001000;
  volatile uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  volatile uint32_t *tpiu_protocol  = (uint32_t *) 0xE00400F0;

  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

  /* Set correct location */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | BSP_TRACE_SWO_LOCATION;

  /* Enable output on correct pin. */
  TRACE_ENABLE_PINS();

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY)) ;

  /* Enable trace in core debug */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;

  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;

  /* Set protocol to NRZ */
  *tpiu_protocol = 2;

  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}


/**************************************************************************//**
 * @brief  Profiler configuration.
 * @return true if energyAware Profiler/SWO is enabled, false if not
 * @note   If first word of the user page is zero, this will not
 *         enable SWO profiler output.
 *****************************************************************************/
bool BSP_TraceProfilerSetup(void)
{
  volatile uint32_t *userData = (uint32_t *) USER_PAGE;

  /* Check magic "trace" word in user page */
  if (*userData == 0x00000000UL)
  {
    return false;
  }
  else
  {
    BSP_TraceSwoSetup();
    return true;
  }
}
