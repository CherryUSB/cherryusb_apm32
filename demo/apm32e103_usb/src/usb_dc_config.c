
#include "main.h"
#include "usb_config.h"
#include "apm32e10x_eint.h"
#include "apm32e10x_gpio.h"
#include "apm32e10x_rcm.h"
#include "apm32e10x_misc.h"
#include "apm32e10x_usb.h"




void usb_dc_low_level_init(void)
{
    EINT_Config_T EINT_ConfigStruct;
    
    RCM_ConfigUSBCLK(RCM_USB_DIV_1_5);
    RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USB);
    
    EINT_ConfigStruct.mode = EINT_MODE_INTERRUPT;
    EINT_ConfigStruct.line = EINT_LINE_18;
    EINT_ConfigStruct.trigger = EINT_TRIGGER_RISING;
    EINT_ConfigStruct.lineCmd = ENABLE;
    EINT_Config(&EINT_ConfigStruct);
    
#if USB_SELECT == USB1
    NVIC_EnableIRQRequest(USBD1_LP_CAN1_RX0_IRQn, 2, 0);  
#else
    NVIC_EnableIRQRequest(USBD2_LP_CAN2_RX0_IRQn, 2, 0);
#endif    
    NVIC_EnableIRQRequest(USBDWakeUp_IRQn, 1, 0);  
    
#if USB_SELECT == USB1
    USBD2_Disable();
#else
    USBD2_Enable();
#endif
    
}

void usb_dc_low_level_deinit(void)
{    
#if USB_SELECT == USB1
    NVIC_DisableIRQRequest(USBD1_LP_CAN1_RX0_IRQn);
#else
    NVIC_DisableIRQRequest(USBD2_LP_CAN2_RX0_IRQn);
#endif    
    NVIC_DisableIRQRequest(USBDWakeUp_IRQn);
}



