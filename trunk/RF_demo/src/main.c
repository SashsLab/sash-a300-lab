/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : simple LED blink demo for EFM32LG_DK3650 and EFM32GG_DK3750
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO
**                Energy Micro peripheral module library for
**                "EFM32" microcontrollers
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "efm32.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_dbg.h"
#include "dvk.h"
#include "trace.h"

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "trace.h"
#include "vars.h"

// i2c driver for Sht75
#include "em_i2c.h"

#include "em_int.h"
#include "em_rtc.h"

#include "segmentlcd.h"
#include "sensirion.h"


/* LED driver */
#include "leds.h"


static MCU_STATES state;
volatile uint32_t msTicks; /* counts 1ms timeTicks */

void Delay(uint32_t dlyTicks);

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  int i = 0;

  uint16_t temp_humid_sensor;
  float rel_humidity, dew_point;
  char buffer_humid[10];
//  char buffer_voltage[10];
//  char light_level[10];


  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  TRACE_ProfilerSetup();

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  /* Disable usart0 clock, it is enabled by default in gecko series mcu's. */
  CMU_ClockEnable(cmuClock_USART0, false);


  /* Infinite blink loop with main state machine */

  state = RESET;

  while (1)
  {
	  //	  state machine

	  switch(state){

	  case RESET:										// RESET state check for vital HW functions

		  if(1){
	  		// todo all checks
	  		i = 0;
	  		state = INIT;
		  }
		  else
			state = RESET;
		  break;

	  case INIT:										// INIT state initialize all necessary peripherials

		  if(1){
			i = i + 1;

			/* Enable clocks. */
			CMU_ClockEnable(cmuClock_GPIO, true);
		    CMU_ClockEnable(cmuClock_I2C0, true);

		    /* Configure PB9, PB10 pin interrupt on falling edge do not enable yet. */
		    GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
		    GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
		    GPIO_IntConfig(gpioPortB, 9, false, true, true);
		    GPIO_IntConfig(gpioPortB, 10, false, true, true);

		    HumiditySensorInit();

			LED_Init();									/* Initialize LED driver */
			LED_Set(0);

			/* Enable LCD without voltage boost */
			SegmentLCD_Init(false);						/* Init LCD driver */
	  		SegmentLCD_Write("ST: INIT");
	  		Delay(500);
	  		SegmentLCD_Number(i);
	  		Delay(500);
	  		state =  IDLE;
		}
		else
		  state = INIT;
	 	  break;

	  case IDLE:
		if(1){
			i = i + 1;
	  		SegmentLCD_Write("ST: IDLE");
	  		Delay(500);
	  		SegmentLCD_Number(i);
	  		Delay(500);

	  		// while idle check humidity and temp paramters
	  	    rel_humidity = ReadHumiditySensor();
	  	    dew_point = getDewPoint();
	  	    /* Get temperature from humidity sensor, notice that both pressure sensor and */
	  	    /* humidity sensor has built in temperature sensors, since dewpoint and pressure */
	  	    /* is dependent on the temperature close to the sensor it gives the most accurate */
	  	    /* results to use the built in temperature sensor in each device. */
	  	    temp_humid_sensor = getTemperatureHumidSensor();


	  		state = RX_STATE;
			}
		else
		  state = IDLE;
		break;

	  case RX_STATE :
		if(1){
			i = i + 1;
	  		SegmentLCD_Write("ST: Rx");
	  		Delay(500);
	  		SegmentLCD_Number(i);
	  		Delay(500);
	  		SegmentLCD_Symbol(LCD_SYMBOL_ANT, 1);
	  		state = TX_STATE;
    		}
		else
		  state = RX_STATE;
		  break;

	  case TX_STATE:
		if(1){
			i = i + 1;
	  		SegmentLCD_Write("ST: TX");
	  		Delay(500);
	  		SegmentLCD_Number(i);
	  		Delay(500);
	  		SegmentLCD_Symbol(LCD_SYMBOL_ANT, 1);
	  		state = ERROR;
		  	}
		else
		  state = TX_STATE;
          break;


	  case ERROR:
		if(1){
			i = i + 1;
	  		SegmentLCD_Write("ST: ERR");
	  		Delay(500);
	  		SegmentLCD_Number(i);
	  		Delay(500);
	  		state = IDLE;
		}
		else
		  state = ERROR;
		  break;
  }
	  LED_Toggle(0);
	  LED_Toggle(1);
	  Delay(1000);
  	}
}
