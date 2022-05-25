/*!
 * @file        main.c
 *
 * @brief       Main program body
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */
#include "stdio.h"
#include "main.h"

extern uint32_t g_time_tick;

int fputc(int ch,FILE *f)
{
    uint8_t c;
    c = ch&0xff;
    while(USART_ReadStatusFlag(USART1, USART_FLAG_TXBE) == RESET);
    USART_TxData(USART1,c);
	return ch;
}

/*!
 * @brief       Main program   
 *
 * @param       None
 *
 * @retval      None
 *
 */
int main(void)
{
    GPIO_Config_T  gpio_cfg_struct;
    USART_Config_T uart_cfg_struct;
    
    NVIC_ConfigPriorityGroup(NVIC_PRIORITY_GROUP_2);
    
    /* Enable the GPIO_LED Clock */
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_GPIOE|RCM_APB2_PERIPH_GPIOA|RCM_APB2_PERIPH_AFIO|RCM_APB2_PERIPH_USART1);

    /* Configure the GPIO_LED pin */
    gpio_cfg_struct.pin = GPIO_PIN_6|GPIO_PIN_5;
    gpio_cfg_struct.mode = GPIO_MODE_OUT_PP;
    gpio_cfg_struct.speed = GPIO_SPEED_50MHz;
    GPIO_Config(GPIOE, &gpio_cfg_struct);
    GPIOE->BC = GPIO_PIN_5|GPIO_PIN_6;

    //
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_GPIOA|RCM_APB2_PERIPH_AFIO|RCM_APB2_PERIPH_USART1);
    /* Configure USART Tx as alternate function push-pull */
    gpio_cfg_struct.mode  = GPIO_MODE_AF_PP;
    gpio_cfg_struct.pin   = GPIO_PIN_9;
    gpio_cfg_struct.speed = GPIO_SPEED_50MHz;
    GPIO_Config(GPIOA, &gpio_cfg_struct);

    /* Configure USART Rx as input floating */
    gpio_cfg_struct.mode = GPIO_MODE_IN_FLOATING;
    gpio_cfg_struct.pin = GPIO_PIN_10;
    GPIO_Config(GPIOA, &gpio_cfg_struct);
    
    USART_ConfigStructInit(&uart_cfg_struct);
    uart_cfg_struct.baudRate = 115200;
    USART_Config(USART1,&uart_cfg_struct);
    USART_Enable(USART1);
    printf("hello\n");
    SysTick_Config(SystemCoreClock/1000);

//////////////////////////////////////////////////////////
//test example


//    extern void hid_mouse_init(void);
//    hid_mouse_init();

//    extern void msc_ram_init(void);
//    msc_ram_init();

extern void cdc_acm_msc_init(void);
    cdc_acm_msc_init();

    while(1)
    {
//        extern void hid_mouse_send_report_test(void);
//        hid_mouse_send_report_test();

        
        if(g_time_tick == 0)
        {
            g_time_tick = 2000;
            extern void cdc_acm_data_send_with_dtr_test(void);
            cdc_acm_data_send_with_dtr_test();
        }        
    }
}

