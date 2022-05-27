
#include "main.h"
#include "usb_config.h"
#include "apm32f0xx.h"
#include "apm32f0xx_pmu.h"
#include "apm32f0xx_eint.h"
#include "apm32f0xx_gpio.h"
#include "apm32f0xx_rcm.h"
#include "apm32f0xx_misc.h"


void usb_connect(void)
{
    USBD->BCD_B.DPPUCTRL = 1;
}

void usb_disconnect(void)
{
    USBD->BCD_B.DPPUCTRL = 0;
}

void usb_dc_low_level_init(void)
{
    usb_disconnect();
    RCM_EnableHSI48();
    RCM_ConfigUSBCLK(RCM_USBCLK_HSI48);
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USB);
    NVIC_EnableIRQRequest(USBD_IRQn, 2);
    usb_connect();
}

void usb_dc_low_level_deinit(void)
{  
    NVIC_DisableIRQRequest(USBD_IRQn);
    usb_disconnect();
}
