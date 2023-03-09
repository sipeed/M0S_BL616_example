#pragma once

#include "usbd_core.h"
#include "usbd_cdc.h"

#include "cfg.h"

#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

/*!< global descriptor */
static const uint8_t usbd_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x0E,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'S', 0x00,                  /* wcChar0 */
    'I', 0x00,                  /* wcChar1 */
    'P', 0x00,                  /* wcChar2 */
    'E', 0x00,                  /* wcChar3 */
    'E', 0x00,                  /* wcChar4 */
    'D', 0x00,                  /* wcChar5 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x2E,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'U', 0x00,                  /* wcChar0 */
    'S', 0x00,                  /* wcChar1 */
    'B', 0x00,                  /* wcChar2 */
    'D', 0x00,                  /* wcChar3 */
    ' ', 0x00,                  /* wcChar4 */
    'C', 0x00,                  /* wcChar5 */
    'D', 0x00,                  /* wcChar6 */
    'C', 0x00,                  /* wcChar7 */
    ' ', 0x00,                  /* wcChar8 */
    'A', 0x00,                  /* wcChar9 */
    'C', 0x00,                  /* wcChar10 */
    'M', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    'W', 0x00,                  /* wcChar13 */
    'I', 0x00,                  /* wcChar14 */
    'T', 0x00,                  /* wcChar15 */
    'H', 0x00,                  /* wcChar16 */
    ' ', 0x00,                  /* wcChar17 */
    'U', 0x00,                  /* wcChar18 */
    'A', 0x00,                  /* wcChar19 */
    'R', 0x00,                  /* wcChar20 */
    'T', 0x00,                  /* wcChar21 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '3', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '0', 0x00,                  /* wcChar6 */
    '9', 0x00,                  /* wcChar7 */
    '1', 0x00,                  /* wcChar8 */
    '1', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};