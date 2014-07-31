/***************************************************************************//**
 * @file
 * @brief Board support package API implementation STK's.
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
#include <string.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "bsp.h"
#if defined( BSP_STK_USE_EBI )
#include "em_ebi.h"
#endif

#if defined( BSP_STK )

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

static USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;

static bool PacketReceive(STK_Packet *pkt);
static void PacketSend(STK_Packet *pkt);

/** @endcond */

/***************************************************************************//**
 * @addtogroup BSP
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup BSP_STK API for STK's
 * @{
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
/**************************************************************************//**
 * @brief Deinitialize board support package functionality.
 *        Reverse actions performed by BSP_Init().
 *
 * @note This functions is currently a dummy.
 *
 * @return
 *   @ref BSP_STATUS_NOT_IMPLEMENTED
 *****************************************************************************/
int BSP_Disable(void)
{
  /* Disable according to what was originally enabled ? */

  return BSP_STATUS_NOT_IMPLEMENTED;
}
/** @endcond */

/**************************************************************************//**
 * @brief Initialize the EBI interface for accessing the onboard nandflash.
 *
 * @note This function is only relevant for STK3700 and STK3600.
 *
 * @return
 *   @ref BSP_STATUS_OK or @ref BSP_STATUS_NOT_IMPLEMENTED
 *****************************************************************************/
int BSP_EbiInit(void)
{
#if defined( BSP_STK_USE_EBI )
  /* ------------------------------------------ */
  /* NAND Flash, Bank0, Base Address 0x80000000 */
  /* Micron flash NAND256W3A                    */
  /* ------------------------------------------ */

  EBI_Init_TypeDef ebiConfig =
  {   ebiModeD8A8,       /* 8 bit address, 8 bit data */
      ebiActiveLow,      /* ARDY polarity */
      ebiActiveLow,      /* ALE polarity */
      ebiActiveLow,      /* WE polarity */
      ebiActiveLow,      /* RE polarity */
      ebiActiveLow,      /* CS polarity */
      ebiActiveLow,      /* BL polarity */
      false,             /* disble BL */
      true,              /* enable NOIDLE */
      false,             /* disable ARDY */
      true,              /* disable ARDY timeout */
      EBI_BANK0,         /* enable bank 0 */
      0,                 /* no chip select */
      0,                 /* addr setup cycles */
      0,                 /* addr hold cycles */
      false,             /* disable half cycle ALE strobe */
      0,                 /* read setup cycles */
      2,                 /* read strobe cycles */
      1,                 /* read hold cycles */
      false,             /* disable page mode */
      false,             /* disable prefetch */
      false,             /* disable half cycle REn strobe */
      0,                 /* write setup cycles */
      2,                 /* write strobe cycles */
      1,                 /* write hold cycles */
      false,             /* enable the write buffer */
      false,             /* disable half cycle WEn strobe */
      ebiALowA24,        /* ALB - Low bound, address lines */
      ebiAHighA26,       /* APEN - High bound, address lines */
      ebiLocation1,      /* Use Location 1 */
      true,              /* enable EBI */
  };

  /* Enable clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_EBI, true);

  /* Enable GPIO's */
  /* ALE and CLE */
  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 0);

  /* WP, CE and R/B */
  GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, 0);   /* active low write-protect */
  GPIO_PinModeSet(gpioPortD, 14, gpioModePushPull, 1);   /* active low chip-enable */
  GPIO_PinModeSet(gpioPortD, 15, gpioModeInput, 0);      /* ready/busy */

  /* IO pins */
  GPIO_PinModeSet(gpioPortE, 8, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 9, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 11, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 12, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 13, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 14, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 15, gpioModePushPull, 0);

  /* WE and RE */
  GPIO_PinModeSet(gpioPortF, 8, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortF, 9, gpioModePushPull, 1);

  /* NAND Power Enable */
  GPIO_PinModeSet(gpioPortB, 15, gpioModePushPull, 1);

  EBI_Init(&ebiConfig);
  EBI->NANDCTRL = (EBI_NANDCTRL_BANKSEL_BANK0 | EBI_NANDCTRL_EN);

  return BSP_STATUS_OK;
#else
  return BSP_STATUS_NOT_IMPLEMENTED;
#endif
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
/**************************************************************************//**
 * @brief Initialize board support package functionality.
 *
 * @param[in] flags Initialization mask, use 0 or @ref BSP_INIT_STK_BCUART.
 *
 * @return
 *   @ref BSP_STATUS_OK
 *****************************************************************************/
int BSP_Init(uint32_t flags)
{
  if ( flags & BSP_INIT_STK_BCUART )
  {
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Configure GPIO pin for USART TX */
    /* To avoid false start, configure output as high. */
    GPIO_PinModeSet(BSP_BC_USART_TXPORT, BSP_BC_USART_TXPIN,
                    gpioModePushPull, 1);

    /* Configure GPIO pin for USART RX */
    GPIO_PinModeSet(BSP_BC_USART_RXPORT, BSP_BC_USART_RXPIN,
                    gpioModeInput, 1);

    /* Enable switch U602A "VMCU switch" - to enable USART communication. */
    /* See board schematics for details. */
    GPIO_PinModeSet(BSP_BC_U602A_PORT, BSP_BC_U602A_PIN,
                    gpioModePushPull, 1);

    CMU_ClockEnable(BSP_BC_USART_CLK, true);

    /* Initialize USART */
    USART_InitAsync(BSP_BC_USART, &usartInit);

    /* Enable correct USART location. */
    BSP_BC_USART->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
                          BSP_BC_USART_LOCATION;
  }

  return BSP_STATUS_OK;
}
/** @endcond */

/**************************************************************************//**
 * @brief Request AEM (Advanced Energy Monitoring) current from board controller.
 *
 * @note Assumes that BSP_Init() has been called with @ref BSP_INIT_STK_BCUART
 *       bitmask.
 *
 * @return
 *   The current expressed in milliamperes. Returns 0.0 on board controller
 *   communication error.
 *****************************************************************************/
float BSP_CurrentGet(void)
{
  STK_Packet pkt;
  float      current;

  pkt.type          = STK_PACKETTYPE_CURRENT_REQ;
  pkt.payloadLength = 0;

  /* Send Request/Get reply */
  PacketSend(&pkt);
  PacketReceive(&pkt);

  /* Process reply */
  if (pkt.type == STK_PACKETTYPE_CURRENT_REPLY)
  {
    memcpy(&current, pkt.data, sizeof(float));
    return current;
  }
  else
  {
    return (float) 0.0;
  }
}

/**************************************************************************//**
 * @brief Request AEM (Advanced Energy Monitoring) voltage from board controller.
 *
 * @note Assumes that BSP_Init() has been called with @ref BSP_INIT_STK_BCUART
  *      bitmask.
 *
 * @return
 *   The voltage. Returns 0.0 on board controller communication
 *   error.
 *****************************************************************************/
float BSP_VoltageGet(void)
{
  STK_Packet pkt;
  float      voltage;

  pkt.type          = STK_PACKETTYPE_VOLTAGE_REQ;
  pkt.payloadLength = 0;

  /* Send Request/Get reply */
  PacketSend(&pkt);
  PacketReceive(&pkt);

  /* Process reply */
  if (pkt.type == STK_PACKETTYPE_VOLTAGE_REPLY)
  {
    memcpy(&voltage, pkt.data, sizeof(float));
    return voltage;
  }
  else
  {
    return (float) 0.0;
  }
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

static bool PacketReceive(STK_Packet *pkt)
{
  uint8_t *rxPtr = (uint8_t *) pkt;
  int     length;

  /* Receive packet magic */
  while (!(BSP_BC_USART->STATUS & USART_STATUS_RXDATAV)) ;
  *rxPtr++ = BSP_BC_USART->RXDATA;
  if (pkt->magic != STK_MAGIC)
  {
    /* Invalid packet */
    memset(pkt, 0x00, sizeof(STK_Packet));
    return false;
  }

  /* Receive packet type */
  while (!(BSP_BC_USART->STATUS & USART_STATUS_RXDATAV)) ;
  *rxPtr++ = BSP_BC_USART->RXDATA;
  if ((pkt->type < STK_PACKETTYPE_FIRST) || (pkt->type > STK_PACKETTYPE_LAST))
  {
    /* Invalid packet */
    memset(pkt, 0x00, sizeof(STK_Packet));
    return false;
  }

  /* Receive packet length */
  while (!(BSP_BC_USART->STATUS & USART_STATUS_RXDATAV)) ;
  *rxPtr++ = BSP_BC_USART->RXDATA;
  if (pkt->payloadLength > STK_PACKET_SIZE)
  {
    /* Invalid packet */
    memset(pkt, 0x00, sizeof(STK_Packet));
    return false;
  }

#if ( BSP_STK_BCP_VERSION == 2 )
  /* Receive reserved byte */
  while (!(BSP_BC_USART->STATUS & USART_STATUS_RXDATAV)) ;
  *rxPtr++ = BSP_BC_USART->RXDATA;
#endif

  /* Receive packet data */
  length = pkt->payloadLength;
  if (length > STK_PACKET_SIZE)
  {
    length = STK_PACKET_SIZE;
  }
  while (length)
  {
    while (!(BSP_BC_USART->STATUS & USART_STATUS_RXDATAV)) ;
    *rxPtr++ = BSP_BC_USART->RXDATA;
    length--;
  }

  return true;
}

__STATIC_INLINE void txByte(uint8_t data)
{
  /* Check that transmit buffer is empty */
  while (!(BSP_BC_USART->STATUS & USART_STATUS_TXBL)) ;
  BSP_BC_USART->TXDATA = (uint32_t) data;
}

static void PacketSend(STK_Packet *pkt)
{
  int i;

  /* Apply magic */
  pkt->magic = STK_MAGIC;

  /* Transmit packet magic */
  txByte(pkt->magic);
  /* Transmit packet type */
  txByte(pkt->type);
  /* Transmit packet length */
  txByte(pkt->payloadLength);

#if ( BSP_STK_BCP_VERSION == 2 )
  /* Transmit reserved byte */
  txByte(pkt->reserved);
#endif

  /* Transmit packet payload */
  for (i = 0; i < pkt->payloadLength; i++)
  {
    txByte(pkt->data[i]);
  }
}

/** @endcond */
/** @} (end group BSP_STK) */
/** @} (end group BSP) */
#endif /* BSP_STK */
