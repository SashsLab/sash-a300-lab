/**************************************************************************//**
 * @file sensirion.c
 * @brief Sensirion SHT75 temp and humidity sensor driver, gpio bit-banged interface
 * @version 1.00
 ******************************************************************************
 * @section License
 ******************************************************************************
 *
 * Copyright (c) 2011 Anders Guldahl
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * - Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the
 *  distribution.
 * - Neither the name of the copyright holder nor the names of
 *  its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Anders Guldahl
 * Modified by Energy Micro (2012).
 *****************************************************************************/
#include <stdio.h>
#include <math.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"

#include "em_gpio.h"

#include "rtc.h"
#include "sensirion.h"

uint16_t temp;
float RH_linear;
float RH_true;
float T; 
float T_dew;

float T_d1 = -39.6;
float T_d2 = 0.04;

float RH_C1 = -2.0468;
float RH_C2 = 0.5872;
float RH_C3 = -0.00040845;

float RH_t1 = 0.01;
float RH_t2 = 0.00128;

float DP_tn = 243.12;
float DP_m = 17.62;

float DP_tn_l = 272.62;
float DP_m_l = 22.46;


uint16_t getTemperature(void);
uint16_t getHumidity(void);


void statusRegWrite(int heater);

  

void HumiditySensorInit(void){

  CMU_ClockEnable(cmuClock_GPIO, true);  
  
  /* power up sensor */
  GPIO_PinModeSet(VDD_PORT, VDD_PIN, gpioModePushPull, 1); /* Vdd */
  GPIO_PinModeSet(DATA_PORT, DATA, gpioModeWiredAndPullUp, 1); /* Data */
  GPIO_PinModeSet(DATA_PORT, SCK, gpioModePushPull, 0); /* SCK */
  
  /* wait for sensor to become ready */
  RTC_Trigger(50, NULL);
  EMU_EnterEM2(false);
  
  /* write status register to change accuracy */
  statusRegWrite(0);
}

float ReadHumiditySensor(void){
  
  uint16_t humidity;
  
  temp = getTemperature();
  humidity = getHumidity();
 
  RH_linear =  RH_C1 + RH_C2 * humidity + RH_C3 * humidity * humidity;
  T = T_d1 + temp * T_d2;
  RH_true = (T-25.0) * (RH_t1 + RH_t2 * humidity) + RH_linear;

  return RH_true;
} 

float getDewPoint(void){
  
    if(T>=0){
      T_dew = DP_tn * ((logf(RH_true/100.0) + ((DP_m*T)/(DP_tn + T)))/(DP_m - logf(RH_true/100.0) - ((DP_m*T)/(DP_tn + T))));
    }else{
      T_dew = DP_tn_l * ((logf(RH_true/100.0) + ((DP_m_l*T)/(DP_tn_l + T)))/(DP_m_l - logf(RH_true/100.0) - ((DP_m_l*T)/(DP_tn_l + T))));  
    }  
    
    return T_dew;

}
          
int getTemperatureHumidSensor(void){

  return (temp*4-3960)/10;
}

uint16_t getTemperature(void){
  
  uint16_t result = 0;
  uint16_t CRC_checksum = 0;
 
  /* Send command */
  
  /* transmission start */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* command 000 00011 */
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK; 
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C4
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C3
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
 
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* check ack */
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //ack
  if( !((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) ){
  
  }
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  /* wait for measurement to be ready */
  RTC_Trigger(100, NULL);

  EMU_EnterEM2(false);

  
  /* Read out data and check CRC */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 15
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 15;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 14
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 14;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 13
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 13;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 12
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 12;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 11
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 11;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 10
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 10;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 9
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 9;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 8
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 8;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA; //ack
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 7
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 7;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 6
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 6;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 5
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 5;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 4
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 4;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 3
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 3;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 2
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 2;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 1
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 1;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 0
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 0;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA; //ack
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  
  /* CRC */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 7
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 7;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 6
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 6;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 5
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 5;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 4
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 4;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 3
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 3;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 2
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 2;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 1
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 1;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 0
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 0;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;    
    
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //nack 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  
  
  /* return temp if crc good, return 0xffff if crc bad */
  return result;
}

uint16_t getHumidity(void){
  
  uint16_t result = 0;
  uint16_t CRC_checksum = 0;
  /* Send command */
  
  /* transmission start */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* command 000 00101 */
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK; 
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C4
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C3
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
 
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* check ack */
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //ack
  if( !((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) ){
  
  }
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  /* wait for measurement to be ready */
  RTC_Trigger(100, NULL);

  EMU_EnterEM2(false);
  
  
  /* Read out data and check CRC */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 15
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 15;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 14
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 14;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 13
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 13;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 12
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 12;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 11
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 11;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 10
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 10;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 9
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 9;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 8
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 8;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA; //ack
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 7
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 7;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 6
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 6;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 5
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 5;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 4
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 4;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 3
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 3;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 2
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 2;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 1
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 1;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 0
  result |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 0;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA; //ack
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  
  /* CRC */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 7
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 7;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 6
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 6;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 5
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 5;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 4
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 4;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 3
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 3;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 2
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 2;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 1
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 1;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit 0
  CRC_checksum |= ((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) << 0;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;    
    
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //nack 
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  
  
  /* return temp if crc good, return 0xffff if crc bad */
  return result;

}


void statusRegWrite(int heater){
  
  /* transmission start */
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK;
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* command 000 00110 */
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK; 
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C4
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C3
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
   
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* check ack */
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //ack
  if( !((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) ){
  
  }
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
  
  /* register 000 00001 */
  GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK; 
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit A0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C4
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C3
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  if(heater){
    GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  }
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C2
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  if(heater){
    GPIO->P[DATA_PORT].DOUTCLR = 1 << DATA;
  }
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C1
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << DATA;
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //bit C0
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;
  
  /* check ack */
  
  GPIO->P[DATA_PORT].DOUTSET = 1 << SCK; //ack
  if( !((GPIO->P[DATA_PORT].DIN >> DATA) & 0x1) ){
  
  }
  GPIO->P[DATA_PORT].DOUTCLR = 1 << SCK;  
}