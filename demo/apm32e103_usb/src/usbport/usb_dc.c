
#include "usb_power.h"
#include "usbd_core.h"


/** Select USB peripheral
*   USB1:   Share FIFO with CAN1
*   USB2:   Private FIFO.Not share whith CAN1
*/
#define USB_SELECT                  USB1

#ifndef USB_NUM_BIDIR_ENDPOINTS
#define USB_NUM_BIDIR_ENDPOINTS     8
#endif

static volatile uint16_t ep_buff_addr;  /* usb endpoint buffer address */

/* Endpoint state */
struct usb_dc_ep_state {
    /** Endpoint max packet size */
    uint16_t ep_mps;
    uint16_t ep_buff_addr;
    /** Endpoint Transfer Type.
     * May be Bulk, Interrupt, Control or Isochronous
     */
    uint8_t ep_type;
    uint8_t ep_stalled; /** Endpoint stall flag */
    
};

/* Driver endpoint state */
struct usb_dc_config_private {
    volatile uint8_t dev_addr;  /* usb address */
    volatile uint8_t ep0_status;/* */
    struct usb_dc_ep_state in_ep[USB_NUM_BIDIR_ENDPOINTS];  /*!< IN endpoint parameters*/
    struct usb_dc_ep_state out_ep[USB_NUM_BIDIR_ENDPOINTS]; /*!< OUT endpoint parameters */
} usb_dc_cfg;


__WEAK void usb_dc_low_level_init(void)
{
}

__WEAK void usb_dc_low_level_deinit(void)
{
}

int usb_dc_init(void)
{
    uint32_t i;
    EINT_Config_T EINT_ConfigStruct;
    
    memset(&usb_dc_cfg, 0, sizeof(struct usb_dc_config_private));
    usb_dc_cfg.in_ep[0].ep_mps     = USB_CTRL_EP_MPS;
    usb_dc_cfg.in_ep[0].ep_type    = 0;
    usb_dc_cfg.in_ep[0].ep_buff_addr    = 0;
    usb_dc_cfg.out_ep[0].ep_mps    = USB_CTRL_EP_MPS;
    usb_dc_cfg.out_ep[0].ep_type   = 0;
    usb_dc_cfg.out_ep[0].ep_buff_addr    = 0;
    usb_dc_cfg.dev_addr = 0;
    ep_buff_addr = 0x40;
    
    usb_dc_low_level_init();
    
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
    NVIC_EnableIRQRequest(USB2_LP_IRQn, 2, 0);
#endif    
    NVIC_EnableIRQRequest(USBDWakeUp_IRQn, 1, 0);  
    
#if USB_SELECT == USB1
    USBD2_Disable();
#else
    USB2_Enable();
#endif
    USB_PowerOn();
    
    USBD_SetBufferTable(USB_BUFFER_TABLE_ADDR);
    //endpoint 0 init
    USBD_ResetEPKind(USBD_EP_0);
    
    USBD_Enable();
    return 0;
}

int usb_dc_deinit(void)
{
    usb_dc_low_level_deinit();
    USB_PowerOff();
    return 0;
}

/*
*   Set Address
*/
int usbd_set_address(const uint8_t addr)
{
//    USB_LOG_DBG("setaddr:%d\r\n", addr);
    if(addr == 0)
    {
        USBD_SetDeviceAddr(0);
    }
    usb_dc_cfg.dev_addr = addr;
    return 0;
}

int usbd_ep_open(const struct usbd_endpoint_cfg *ep_cfg)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->ep_addr);

    if(ep_idx >= USB_NUM_BIDIR_ENDPOINTS)   return -1;
        
    USBD_SetEpAddr((USBD_EP_T)ep_idx, ep_idx);
    switch (ep_cfg->ep_type)
    {
        case    USB_ENDPOINT_TYPE_CONTROL:
            USBD_SetEPType(ep_idx, USBD_EP_TYPE_CONTROL);
            break;
        case    USB_ENDPOINT_TYPE_ISOCHRONOUS:
            USBD_SetEPType(ep_idx, USBD_EP_TYPE_ISO);
            break;
        case    USB_ENDPOINT_TYPE_BULK:
            USBD_SetEPType(ep_idx, USBD_EP_TYPE_BULK);
            break;
        case    USB_ENDPOINT_TYPE_INTERRUPT:
            USBD_SetEPType(ep_idx, USBD_EP_TYPE_INTERRUPT);
            break;
        default:
            break;
    }
        
    if (USB_EP_DIR_IS_OUT(ep_cfg->ep_addr))
    {
        //out endpoint
        usb_dc_cfg.out_ep[ep_idx].ep_mps = ep_cfg->ep_mps;
        usb_dc_cfg.out_ep[ep_idx].ep_type = ep_cfg->ep_type;
        usb_dc_cfg.out_ep[ep_idx].ep_stalled = USBD_EP_STATUS_VALID;
        if(usb_dc_cfg.out_ep[ep_idx].ep_buff_addr == 0)
        {
            usb_dc_cfg.out_ep[ep_idx].ep_buff_addr = ep_buff_addr;
            ep_buff_addr += ep_cfg->ep_mps;
            ep_buff_addr = ((ep_buff_addr+3)>>2)<<2;    //4byte algin
        }
        USBD_SetEPRxAddr(ep_idx, usb_dc_cfg.out_ep[ep_idx].ep_buff_addr);
        USBD_SetEPRxCnt(ep_idx, ep_cfg->ep_mps);
        USBD_SetEPRxStatus(ep_idx, USBD_EP_STATUS_VALID);
        
        USB_LOG_DBG("epopen:0x%02X,0x%X,%d\r\n", ep_cfg->ep_addr,
                                        usb_dc_cfg.out_ep[ep_idx].ep_buff_addr,
                                        ep_cfg->ep_mps);
    } else 
    {
        //in point
        usb_dc_cfg.in_ep[ep_idx].ep_mps = ep_cfg->ep_mps;
        usb_dc_cfg.in_ep[ep_idx].ep_type = ep_cfg->ep_type;
        usb_dc_cfg.in_ep[ep_idx].ep_stalled = USBD_EP_STATUS_STALL;
        if(usb_dc_cfg.in_ep[ep_idx].ep_buff_addr == 0)
        {
            usb_dc_cfg.in_ep[ep_idx].ep_buff_addr = ep_buff_addr;
            ep_buff_addr += ep_cfg->ep_mps;
            ep_buff_addr = ((ep_buff_addr+3)>>2)<<2;    //4byte algin            
        }
        USBD_SetEPTxAddr(ep_idx, usb_dc_cfg.in_ep[ep_idx].ep_buff_addr);
        USBD_SetEPTxCnt(ep_idx, ep_cfg->ep_mps);
        USBD_SetEPTxStatus(ep_idx, USBD_EP_STATUS_STALL);
        
        USB_LOG_DBG("epopen:0x%02X,0x%X,%d\r\n", ep_cfg->ep_addr,
                                        usb_dc_cfg.in_ep[ep_idx].ep_buff_addr,
                                        ep_cfg->ep_mps);
    }
    return 0;
}

int usbd_ep_close(const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    
    USB_LOG_DBG("epclose:0x%02X\r\n", ep);
    if (USB_EP_DIR_IS_OUT(ep))
    {
        USBD_SetEPRxStatus(ep_idx, USBD_EP_STATUS_DISABLE);
        usb_dc_cfg.out_ep[ep_idx].ep_stalled = USBD_EP_STATUS_DISABLE;
    } else
    {
        USBD_SetEPTxStatus(ep_idx, USBD_EP_STATUS_DISABLE);
        usb_dc_cfg.in_ep[ep_idx].ep_stalled = USBD_EP_STATUS_DISABLE;
    }
    return 0;
}

int usbd_ep_set_stall(const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

    USB_LOG_DBG("epstall:0x%02X\r\n", ep);
    if (USB_EP_DIR_IS_OUT(ep))
    {
        USBD_SetEPRxStatus(ep_idx, USBD_EP_STATUS_STALL);
        usb_dc_cfg.out_ep[ep_idx].ep_stalled = USBD_EP_STATUS_STALL;
    } else
    {
        USBD_SetEPTxStatus(ep_idx, USBD_EP_STATUS_STALL);
        usb_dc_cfg.in_ep[ep_idx].ep_stalled = USBD_EP_STATUS_STALL;
    }
    return 0;
}

int usbd_ep_clear_stall(const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    
    USB_LOG_DBG("epvalid:0x%02X\r\n", ep);
    if (USB_EP_DIR_IS_OUT(ep))
    {
        USBD_SetEPRxStatus(ep_idx, USBD_EP_STATUS_VALID);
        usb_dc_cfg.out_ep[ep_idx].ep_stalled = USBD_EP_STATUS_VALID;
    } else
    {
        USBD_SetEPTxStatus(ep_idx, USBD_EP_STATUS_VALID);
        usb_dc_cfg.in_ep[ep_idx].ep_stalled = USBD_EP_STATUS_VALID;
    }
    return 0;
}

int usbd_ep_is_stalled(const uint8_t ep, uint8_t *stalled)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
//    USB_LOG_DBG("ep_status:%02X\r\n", ep);
    if (USB_EP_DIR_IS_OUT(ep))
    {
        *stalled = (usb_dc_cfg.out_ep[ep_idx].ep_stalled == USBD_EP_STATUS_STALL);
    } else
    {
        *stalled = (usb_dc_cfg.in_ep[ep_idx].ep_stalled == USBD_EP_STATUS_STALL);
    }
    return 0;
}

int usbd_ep_write(const uint8_t ep, const uint8_t *data, uint32_t data_len, uint32_t *ret_bytes)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);

//    if(ep&0x7f)    USB_LOG_INFO("write:0x%02X:0x%X->%d\r\n", ep, (uint32_t)data, data_len); 
    if (!(ep & 0x80)){
        return -1;
    }
    
    if (!data && data_len) {
        return -1;
    }

    if (!data_len) 
    {
        USBD_SetEPTxCnt(ep_idx, 0);
        USBD_SetEPRxCnt(ep_idx, usb_dc_cfg.out_ep[ep_idx].ep_mps);
        USBD_SetEPRxTxStatus(ep_idx, USBD_EP_STATUS_VALID, USBD_EP_STATUS_VALID);
        usb_dc_cfg.ep0_status = 1;
        return 0;
    }
       
    if (data_len > usb_dc_cfg.in_ep[ep_idx].ep_mps)
    {
        data_len = usb_dc_cfg.in_ep[ep_idx].ep_mps;
    }else   usb_dc_cfg.ep0_status = 1;
    USBD_WriteDataToEP(ep_idx, (uint8_t *)data, data_len);
    USBD_SetEPTxCnt(ep_idx, data_len);
    USBD_SetEPRxTxStatus(ep_idx, USBD_EP_STATUS_VALID,USBD_EP_STATUS_VALID); //

    if (ret_bytes) 
    {
        *ret_bytes = data_len;
    }

    return 0;
}

int usbd_ep_read(const uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint32_t read_count;
    
//    if(ep&0x7f)  USB_LOG_INFO("read:0x%02X:0x%X->%d\r\n", ep, (uint32_t)data, max_data_len);
    if (ep & 0x80){
        return -1;
    }
    
    if (!data && max_data_len)
    {
        return -1;
    }

    if (!max_data_len)
    {
        USBD_SetEPTxCnt(ep_idx, 0);
        USBD_SetEPRxCnt(ep_idx, usb_dc_cfg.out_ep[ep_idx].ep_mps);
        USBD_SetEPRxTxStatus(ep_idx,  USBD_EP_STATUS_VALID , USBD_EP_STATUS_VALID );
        return 0;
    }
    
    USBD_ReadDataFromEP(ep_idx, (uint8_t *)data, max_data_len);
    USBD_SetEPRxStatus(ep_idx, USBD_EP_STATUS_VALID);    //recv data valid
    read_count = max_data_len;
    
    if (read_bytes != NULL)
    {
        *read_bytes = read_count;
    }

    return 0;
}


//#ifndef USBD_IRQHandler
//#define USBD_IRQHandler USB_FS_Handler //use actual usb irq name instead
//#endif
//void USBD_IRQHandler(void)
//{
//}

/*!
 * @brief       USB low priority process       
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USBD_LowPriorityProc(void)
{
    USBD_EP_T ep;
    
    while(USBD_ReadIntFlag(USBD_INT_CTR))
    {
        ep = (USBD_EP_T)USBD_ReadEP();
        
        /** Endpoint 0 */
        if(ep == 0)
        {
            USBD_SetEPRxTxStatus(ep, USBD_EP_STATUS_NAK, USBD_EP_STATUS_NAK);
            
            /** Control in */
            if(USBD_ReadDir() == 0)
            {
                USBD_ResetEPTxFlag(ep);
                USB_LOG_DBG("IN packet\r\n\r\n");
                usbd_event_notify_handler(USBD_EVENT_EP0_IN_NOTIFY, NULL);

                if(usb_dc_cfg.dev_addr)
                {
                    USB_LOG_DBG("usbaddr=%d\r\n\r\n",usb_dc_cfg.dev_addr);
                    USBD_SetDeviceAddr(usb_dc_cfg.dev_addr);
                    usb_dc_cfg.dev_addr = 0;
                    USBD_SetEPRxTxStatus(0,  USBD_EP_STATUS_STALL , USBD_EP_STATUS_STALL );
                }else
                {
                    USBD_SetEPRxTxStatus(0, USBD_EP_STATUS_VALID, USBD_EP_STATUS_VALID);
                }
                
            }else
            {
                USBD_ResetEPRxFlag(ep);
                /** Setup */
                if(USBD_ReadEPSetup(ep) == SET)
                {
                    USB_LOG_DBG("SETUP packet\r\n\r\n");
                    usbd_event_notify_handler(USBD_EVENT_SETUP_NOTIFY, NULL);
                }
                /** Control out */
                else
                {
                    USB_LOG_DBG("OUT packet\r\n\r\n");
                    usbd_event_notify_handler(USBD_EVENT_EP0_OUT_NOTIFY, NULL);
                }
            }
        }else
        {
            /* other endpoint */
            if(USBD_ReadEPRxFlag(ep))   /* recvive out*/
            {
                USBD_ResetEPRxFlag(ep);
                USB_LOG_DBG("OUT 0x%02X\r\n\r\n",ep);
                usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (void *)(ep & 0x7f));
            }
            if(USBD_ReadEPTxFlag(ep))   /*send in */
            {
                USBD_ResetEPTxFlag(ep);
                USB_LOG_DBG("IN 0x%02X\r\n\r\n",ep);
                usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (void *)(ep | 0x80));
            }
        }
    }
}

/*!
 * @brief       USB low priority process 
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USBD_HighPriorityProc(void)
{
    USBD_EP_T ep;
    
    while(USBD_ReadIntFlag(USBD_INT_CTR))
    {
        USBD_ClearIntFlag(USBD_INT_CTR);
        ep = USBD_ReadEP();
        if(USBD_ReadEPRxFlag(ep))
        {
            USBD_ResetEPRxFlag(ep);
            if((ep & 0x7f) == 0)
            {
                usbd_event_notify_handler(USBD_EVENT_EP0_OUT_NOTIFY, NULL);
            }else
            {
                usbd_event_notify_handler(USBD_EVENT_EP_OUT_NOTIFY, (void *)(ep & 0x7f));
            }
        }
        if(USBD_ReadEPTxFlag(ep))
        {
            USBD_ResetEPTxFlag(ep);
            if((ep & 0x7f) == 0)
            {
                usbd_event_notify_handler(USBD_EVENT_EP0_IN_NOTIFY, NULL);
            }else
            {
                usbd_event_notify_handler(USBD_EVENT_EP_IN_NOTIFY, (void *)(ep | 0X80));
            }
        }
    }
}

/*!
 * @brief       USB interrupt service routine
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
#if USB_SELECT == USB1 
void USBD1_LP_CAN1_RX0_IRQHandler(void)
#else
void USB2_LP_IRQHandler(void)
#endif
{
#if (USB_INT_SOURCE & USBD_INT_RST) 
    if(USBD_ReadIntFlag(USBD_INT_RST))
    {
        USBD_ClearIntFlag(USBD_INT_RST);
        USB_LOG_DBG("RESET.....................................\r\n\r\n");
//        USB_Reset();
        usbd_event_notify_handler(USBD_EVENT_RESET, NULL);
    }  
#endif

#if USB_INT_SOURCE & USBD_INT_PMAOU
    if(USB_ReadIntFlag(USB_INT_PMAOU))
    {
        USB_ClearIntFlag(USB_INT_PMAOU);
    }     
#endif

#if USB_INT_SOURCE & USBD_INT_ERR

    if(USB_ReadIntFlag(USB_INT_ERROR))
    {
        USB_ClearIntFlag(USB_INT_ERROR);
    }
#endif

#if USB_INT_SOURCE & USBD_INT_WKUP
    if(USBD_ReadIntFlag(USBD_INT_WKUP))
    {
        USB_Resume();
        USBD_ClearIntFlag(USBD_INT_WKUP);
    }
#endif

#if USB_INT_SOURCE & USBD_INT_SUS
    if(USBD_ReadIntFlag(USBD_INT_SUS))
    {
        USB_Suspend();
        USBD_ClearIntFlag(USBD_INT_SUS);
    }   
#endif

#if USB_INT_SOURCE & USBD_INT_SOF
    if(USB_ReadIntFlag(USB_INT_SOF))
    {
        USB_ClearIntFlag(USB_INT_SOF);
    }    
#endif

#if USB_INT_SOURCE & USBD_INT_ESOF
    if(USB_ReadIntFlag(USB_INT_ESOF))
    {
        USB_ClearIntFlag(USB_INT_ESOF);
    }    
#endif    
    
#if (USB_INT_SOURCE & USBD_INT_CTR)
    if(USBD_ReadIntFlag(USBD_INT_CTR))
    {
        USBD_LowPriorityProc();
    }
#endif    
}

#if USB_SELECT == USB1 
void USBD1_HP_CAN1_TX_IRQHandler(void)
#else
void USB2_HP_IRQHandler(void)
#endif
{
    USBD_HighPriorityProc();
}




