/*!
 * @file        usb_power.c
 *
 * @brief       USB power management       
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */
#include "usb_power.h"
#include "apm32e10x_pmu.h"
#include "apm32e10x_usb.h"

/*!
 * @brief       USB Reset
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USB_Reset(void)
{

}

/*!
 * @brief       USB Power on
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USB_PowerOn(void)
{
    USBD_ResetPowerDown();

    USBD_SetForceReset();
    USBD_ResetForceReset();
    
    USBD_DisableInterrupt(USBD_INT_ALL);
    USBD_ClearIntFlag(USBD_INT_ALL);
    
    USBD_EnableInterrupt(USB_INT_SOURCE);
}

/*!
 * @brief       USB Power off
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USB_PowerOff(void)
{
    USBD_DisableInterrupt(USBD_INT_ALL);
    USBD_ClearIntFlag(USBD_INT_ALL);
    USBD_SetRegCTRL(0X03);
}

/*!
 * @brief       USB Suspend
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USB_Suspend(void)
{
    uint8_t i;
    uint16_t bakEP[8];
    uint32_t bakPwrCR;
    uint32_t tmp;
    
    for(i = 0; i < 8; i++)
    {
        bakEP[i] = (uint16_t)USBD->EP[i].EP;
    }

    USBD_EnableInterrupt(USBD_INT_RST);
    
    USBD_SetForceReset();
    USBD_ResetForceReset();

    while(USBD_ReadIntFlag(USBD_INT_RST) == RESET);

    for(i = 0; i < 8; i++)
    {
        USBD->EP[i].EP = bakEP[i];
    }

    USBD_SetForceSuspend();
    USBD_SetLowerPowerMode();

    bakPwrCR = PMU->CTRL;
    tmp = PMU->CTRL;
    tmp &= (uint32_t)0xfffffffc;
    tmp |= PMU_REGULATOR_LOWPOWER; 
    PMU->CTRL = tmp;

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    if(USBD_ReadIntFlag(USBD_INT_WKUP) == RESET)
    {
//        __WFI();
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    }
    else
    {
        USBD_ClearIntFlag(USBD_INT_WKUP);
        USBD_ResetForceSuspend();
        PMU->CTRL = bakPwrCR;
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    }
}


/*!
 * @brief       Resume
 *
 * @param       None
 *
 * @retval      None
 *
 * @note       
 */
void USB_Resume(void)
{
    USBD_ResetLowerPowerMode();

    SystemInit();
    
    USBD_SetRegCTRL(USB_INT_SOURCE);
}
