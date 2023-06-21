/**
  **************************************************************************
  * @file     cdc_keyboard_desc.h
  * @version  v2.0.6
  * @date     2021-12-31
  * @brief    usb cdc and keyboard descriptor header file
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
#ifndef __CDC_KEYBOARD_DESC_H
#define __CDC_KEYBOARD_DESC_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include "cdc_keyboard_class.h"
#include "usbd_core.h"

/** @addtogroup AT32F403A_407_middlewares_usbd_class
  * @{
  */
  
/** @addtogroup USB_cdc_keyboard_desc
  * @{
  */

/** @defgroup USB_cdc_keyboard_desc_definition 
  * @{
  */

/**
  * @brief usb bcd number define
  */
#define BCD_NUM                          0x0110

/**
  * @brief usb vendor id and product id define
  */
#define USBD_VENDOR_ID                   0x2E3C
#define USBD_PRODUCT_ID                  0x5750 

/**
  * @brief usb bcd number define
  */
#define HID_BCD_NUM                      0x0110

/**
  * @brief usb hid class descriptor define
  */
#define HID_CLASS_DESC_HID               0x21
#define HID_CLASS_DESC_REPORT            0x22
#define HID_CLASS_DESC_PHYSICAL          0x23

/**
  * @brief usb descriptor size define
  */
#define USBD_CONFIG_DESC_SIZE            108
#define USBD_HID_SIZ_REPORT_DESC         63
#define USBD_SIZ_STRING_LANGID           4
#define USBD_SIZ_STRING_SERIAL           0x1A

/**
  * @brief usb string define(vendor, product configuration, interface)
  */
#define USBD_DESC_MANUFACTURER_STRING    "Artery"
#define USBD_DESC_PRODUCT_STRING         "AT32 Composite VCP and Keyboard"
#define USBD_DESC_CONFIGURATION_STRING   "Composite VCP and Keyboard Config"
#define USBD_DESC_INTERFACE_STRING       "Composite VCP and Keyboard Interface"

/**
  * @brief usb endpoint interval define 
  */
#define HID_BINTERVAL_TIME                0x0A

/**
  * @brief usb cdc class descriptor define
  */
#define USBD_CDC_CS_INTERFACE             0x24
#define USBD_CDC_CS_ENDPOINT              0x25

/**
  * @brief usb cdc class sub-type define
  */
#define USBD_CDC_SUBTYPE_HEADER           0x00
#define USBD_CDC_SUBTYPE_CMF              0x01
#define USBD_CDC_SUBTYPE_ACM              0x02
#define USBD_CDC_SUBTYPE_UFD              0x06

/**
  * @brief usb interface define
  */
#define CDC_INTERFACE                    0x00
#define CDC_DATA_INTERFACE               0x01
#define HID_KEYBOARD_INTERFACE           0x02

/**
  * @brief usb mcu id address deine
  */
#define         MCU_ID1                   (0x1FFFF7E8)
#define         MCU_ID2                   (0x1FFFF7EC)
#define         MCU_ID3                   (0x1FFFF7F0)

/**
  * @}
  */
  
extern uint8_t g_usbd_descriptor[USB_DEVICE_DESC_LEN];
extern uint8_t g_usbd_configuration[USBD_CONFIG_DESC_SIZE];
extern uint8_t g_usbd_hid_report[USBD_HID_SIZ_REPORT_DESC];
extern uint8_t g_hid_usb_desc[9];
extern usbd_desc_handler cdc_keyboard_desc_handler;

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
