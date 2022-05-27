#ifndef _CHERRY_USB_CONFIG_H
#define _CHERRY_USB_CONFIG_H


#define USBD_IRQHandler     USBD_IRQHandler //use actual usb irq name instead
#define PMA_ACCESS          1U

/* USB DEVICE Configuration */
/* core */
#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

#ifndef CONFIG_USBDEV_DESC_CHECK
#define CONFIG_USBDEV_DESC_CHECK 0
#endif

#ifndef CONFIG_USBDEV_TEST_MODE
#define CONFIG_USBDEV_TEST_MODE 0
#endif

/* msc class */
#ifndef CONFIG_USBDEV_MSC_MANUFACTURER_STRING
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_PRODUCT_STRING
#define CONFIG_USBDEV_MSC_PRODUCT_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_VERSION_STRING
#define CONFIG_USBDEV_MSC_VERSION_STRING "0.01"
#endif

#ifndef CONFIG_USBDEV_MSC_THREAD_ENABLE
#define CONFIG_USBDEV_MSC_THREAD_ENABLE 0
#endif

#ifndef CONFIG_USBDEV_MSC_STACKSIZE
#define CONFIG_USBDEV_MSC_STACKSIZE 2048
#endif

#ifndef CONFIG_USBDEV_MSC_PRIO
#define CONFIG_USBDEV_MSC_PRIO 10
#endif

/* audio class */
#ifndef CONFIG_USBDEV_AUDIO_VERSION
#define CONFIG_USBDEV_AUDIO_VERSION 0x0100
#endif

#ifndef CONFIG_USBDEV_AUDIO_MAX_CHANNEL
#define CONFIG_USBDEV_AUDIO_MAX_CHANNEL 2
#endif

/* USB HOST Configuration */
#ifndef CONFIG_USBHOST_RHPORTS
#define CONFIG_USBHOST_RHPORTS 1
#endif

#ifndef CONFIG_USBHOST_EHPORTS
#define CONFIG_USBHOST_EHPORTS 4
#endif

#ifndef CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USBHOST_PIPE_NUM 10
#endif

#ifndef CONFIG_USBHOST_INTF_NUM
#define CONFIG_USBHOST_INTF_NUM 6
#endif

#ifndef CONFIG_USBHOST_EP_NUM
#define CONFIG_USBHOST_EP_NUM 4
#endif

#ifndef CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT
#define CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT 5000
#endif

#ifndef CONFIG_USBHOST_MSC_TIMEOUT
#define CONFIG_USBHOST_MSC_TIMEOUT 5000
#endif

#ifndef CONFIG_USBHOST_HPWORKQ_PRIO
#define CONFIG_USBHOST_HPWORKQ_PRIO 5
#endif
#ifndef CONFIG_USBHOST_HPWORKQ_STACKSIZE
#define CONFIG_USBHOST_HPWORKQ_STACKSIZE 2048
#endif

#ifndef CONFIG_USBHOST_LPWORKQ_PRIO
#define CONFIG_USBHOST_LPWORKQ_PRIO 1
#endif
#ifndef CONFIG_USBHOST_LPWORKQ_STACKSIZE
#define CONFIG_USBHOST_LPWORKQ_STACKSIZE 2048
#endif

#ifndef CONFIG_USBHOST_PSC_PRIO
#define CONFIG_USBHOST_PSC_PRIO 4
#endif
#ifndef CONFIG_USBHOST_PSC_STACKSIZE
#define CONFIG_USBHOST_PSC_STACKSIZE 4096
#endif

#ifndef CONFIG_USBHOST_DEV_NAMELEN
#define CONFIG_USBHOST_DEV_NAMELEN 16
#endif

#define CONFIG_USBHOST_ASYNCH
//#define CONFIG_USBHOST_GET_STRING_DESC

/* EHCI Configuration */
#define CONFIG_USB_EHCI_HCCR_BASE (0x20072000)
#define CONFIG_USB_EHCI_HCOR_BASE (0x20072000 + 0x10)
#define CONFIG_USB_EHCI_QH_NUM    (10)
#define CONFIG_USB_EHCI_QTD_NUM   (10)
// #define CONFIG_USB_EHCI_INFO_ENABLE
#define CONFIG_USB_ECHI_HCOR_RESERVED_DISABLE
// #define CONFIG_USB_EHCI_CONFIGFLAG

#endif  /* _CHERRY_USB_CONFIG_H */
