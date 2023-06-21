/**
  **************************************************************************
  * @file     msc_desc.h
  * @version  v2.0.6
  * @date     2021-12-31
  * @brief    usb msc descriptor header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
  
/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSC_DESC_H
#define __MSC_DESC_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include "msc_class.h"
#include "usbd_core.h"

/** @addtogroup AT32F403A_407_middlewares_usbd_class
  * @{
  */
  
/** @addtogroup USB_msc_desc
  * @{
  */

/** @defgroup USB_msc_desc_definition 
  * @{
  */

#define BCD_NUM                          0x0110

#define USBD_VENDOR_ID                   0x2E3C
#define USBD_PRODUCT_ID                  0x5720 

#define USBD_CONFIG_DESC_SIZE            32
#define USBD_SIZ_STRING_LANGID           4
#define USBD_SIZ_STRING_SERIAL           0x1A

#define USBD_DESC_MANUFACTURER_STRING    "Artery"
#define USBD_DESC_PRODUCT_STRING         "AT32 Mass Storage"
#define USBD_DESC_CONFIGURATION_STRING   "Mass Storage Config"
#define USBD_DESC_INTERFACE_STRING       "Mass Storage Interface"

#define HID_BINTERVAL_TIME                0xFF

#define USBD_CDC_CS_INTERFACE             0x24
#define USBD_CDC_CS_ENDPOINT              0x25

#define USBD_CDC_SUBTYPE_HEADER           0x00
#define USBD_CDC_SUBTYPE_CMF              0x01
#define USBD_CDC_SUBTYPE_ACM              0x02
#define USBD_CDC_SUBTYPE_UFD              0x06


#define         MCU_ID1                   (0x1FFFF7E8)
#define         MCU_ID2                   (0x1FFFF7EC)
#define         MCU_ID3                   (0x1FFFF7F0)

extern uint8_t g_usbd_descriptor[USB_DEVICE_DESC_LEN];
extern uint8_t g_usbd_configuration[USBD_CONFIG_DESC_SIZE];
extern usbd_desc_handler msc_desc_handler;

/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif

