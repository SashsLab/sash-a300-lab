/**************************************************************************//**
 * @file
 * @brief LED driver code for Energy Micro EFM32_G8xx_STK starter kit
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

#include "efm32.h"
#include "leds.h"
#include "em_gpio.h"
#include "em_cmu.h"

/** GPIO port E */
#define LEDPORT    gpioPortE
/**************************************************************************//**
 * @brief Light up LED
 * @param led LED number (0-1)
 *****************************************************************************/
void LED_Set(int led)
{
  if ((led >= 0) && (led < NO_OF_LEDS))
  {
    GPIO_PinOutSet(LEDPORT, (led + 2));
  }
}

/**************************************************************************//**
 * @brief Return LED status, on or off
 * @param led LED number (0-1)
 *****************************************************************************/
int LED_Get(int led)
{
  int retVal = 0;

  if ((led >= 0) && (led < NO_OF_LEDS))
  {
    retVal = GPIO_PinOutGet(LEDPORT, (led + 2));
  }
  return retVal;
}

/**************************************************************************//**
 * @brief Turn off LED
 * @param led LED number (0-1)
 *****************************************************************************/
void LED_Clear(int led)
{
  if ((led >= 0) && (led < NO_OF_LEDS))
  {
    GPIO_PinOutClear(LEDPORT, (led + 2));
  }
}

/**************************************************************************//**
 * @brief Toggle LED, switch from on to off or vice versa
 * @param led LED number (0-1)
 *****************************************************************************/
void LED_Toggle(int led)
{
  if ((led >= 0) && (led < NO_OF_LEDS))
  {
    GPIO_PinOutToggle(LEDPORT, (led + 2));
  }
}


/**************************************************************************//**
 * @brief Light up LEDs according value of 2 least significat bits
 * @param value Bit pattern
 *****************************************************************************/
void LED_Value(int value)
{
  /* Set the value directly using 0xc as a mask. */
  GPIO_PortOutSetVal(LEDPORT, (value << 2), 0xc);
}


/**************************************************************************//**
 * @brief Initialize LED interface
 *****************************************************************************/
void LED_Init(void)
{
  /* Enable GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure GPIO port E 2-3 as LED control outputs */
  /* Enable the LED by default */
  GPIO_PinModeSet(LEDPORT, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(LEDPORT, 3, gpioModePushPull, 0);
}
