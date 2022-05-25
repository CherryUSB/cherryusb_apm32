/*!
 * @file        usb_power.h
 *
 * @brief       USB power management       
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */
#ifndef USB_POWER_H_
#define USB_POWER_H_

#include "main.h"
#include "apm32e10x_eint.h"
#include "apm32e10x_gpio.h"
#include "apm32e10x_rcm.h"
#include "apm32e10x_misc.h"
#include "apm32e10x_usb.h"

#define USB1                        0
#define USB2                        1

/** Endpoint pack size in bytes */
#define USB_EP_PACKET_SIZE          64

/** Buffer table address */
#define USB_BUFFER_TABLE_ADDR       0

/** EP0 Tx address */
#define USB_EP0_TX_ADDR         (0X40)
/** EP0 Rx address */
#define USB_EP0_RX_ADDR         (0X80)

/** Interrupt source */
#define USB_INT_SOURCE          (USBD_INT_RST | USBD_INT_CTR | USBD_INT_SUS | USBD_INT_WKUP)


void USB_PowerOn(void);
void USB_PowerOff(void);
void USB_Suspend(void);
void USB_Resume(void);
void USB_Reset(void);

#endif
