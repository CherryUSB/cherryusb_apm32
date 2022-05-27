/*!
 * @file        main.c
 *
 * @brief       Main program body
 *
 * @version     V1.0.0
 *
 * @date        2021-12-30
 *
 * @attention
 *
 *  Copyright (C) 2020-2022 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */
#include "stdio.h"
#include "main.h"


static void SystemClockHSIPLL48M(void)
{
    uint32_t i;

    RCM->CTRL1_B.HSIEN= BIT_SET;
    /** Select HSI as System Clock at first*/
    RCM_ConfigSYSCLK(RCM_SYSCLK_SEL_HSI);
    /** Disable PLL*/
    RCM_DisablePLL();
    /** Wait until Pll is ready*/
    while (RCM->CTRL1_B.PLLRDYFLG != RESET);
    /** Config PLL source and multiplication factor
        SYSCLKFreq = (HSI * 6) / 1*/
    RCM->CFG1_B.PLLSRCSEL = 1;  //PLL input select HSI
    RCM->CFG2_B.PLLDIVCFG = 0;  //PLL input div 1
    RCM->CFG1_B.PLLMULCFG = 4;  //PLL mul 6
    /** Enable PLL*/
    RCM_EnablePLL();
    /** Wait until Pll is ready*/
    while (RCM->CTRL1_B.PLLRDYFLG == RESET);

    if (RCM->CTRL1_B.PLLRDYFLG)
    {
        /** Enable Prefetch Buffer */
        FMC->CTRL1_B.PBEN = BIT_SET;
        /** Flash 1 wait state */
        FMC->CTRL1_B.WS = 1;
        /** HCLK = SYSCLK */
        RCM->CFG1_B.AHBPSC= 0X00;
        /** PCLK = HCLK */
        RCM->CFG1_B.APB1PSC = 0X00;
        /** Select PLL as system clock source */
        RCM->CFG1_B.SCLKSEL = 2;
        /** Wait till PLL is used as system clock source */
        while (RCM->CFG1_B.SCLKSWSTS != 0x02);
    }
    SystemCoreClock = RCM_ReadSYSCLKFreq();
}

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
    GPIO_Config_T   gpioConfig;
    USART_Config_T  usartConfigStruct;
    
    SystemClockHSIPLL48M();
    
    RCM_EnableAHBPeriphClock(RCM_AHB_PERIPH_GPIOA);
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART1);
    
    GPIO_ConfigPinAF(GPIOA,GPIO_PIN_SOURCE_9,GPIO_AF_PIN1);
    GPIO_ConfigPinAF(GPIOA,GPIO_PIN_SOURCE_10,GPIO_AF_PIN1);
    /** Configure USART Tx as alternate function push-pull */
    gpioConfig.mode = GPIO_MODE_AF;
    gpioConfig.pin = GPIO_PIN_9;
    gpioConfig.speed = GPIO_SPEED_50MHz;
    gpioConfig.outtype = GPIO_OUT_TYPE_PP;
    gpioConfig.pupd = GPIO_PUPD_PU;
    GPIO_Config(GPIOA, &gpioConfig);

    /** Configure USART Rx as input floating */
    gpioConfig.pin = GPIO_PIN_10;
    GPIO_Config(GPIOA, &gpioConfig);
    
    usartConfigStruct.baudRate = 115200;
    usartConfigStruct.mode     = USART_MODE_TX_RX;
    usartConfigStruct.hardwareFlowCtrl = USART_FLOW_CTRL_NONE;
    usartConfigStruct.parity   = USART_PARITY_NONE;
    usartConfigStruct.stopBits =  USART_STOP_BIT_1;
    usartConfigStruct.wordLength = USART_WORD_LEN_8B;
    USART_Config(USART1, &usartConfigStruct);
    USART_Enable(USART1);
    
    SysTick_Config(SystemCoreClock/1000);
    
    printf("apm32f072\r\n");
//////////////////////////////////////////////////////////
//test example

//    extern void hid_mouse_init(void);
//    hid_mouse_init();

    extern void msc_ram_init(void);
    msc_ram_init();

//    extern void cdc_acm_init(void);
//    cdc_acm_init();
    
//    extern void cdc_acm_msc_init(void);
//    cdc_acm_msc_init();

    while(1)
    {
//        extern void hid_mouse_send_report_test(void);
//        hid_mouse_send_report_test();

        
//        if(g_time_tick == 0)
//        {
//            g_time_tick = 2000;
//            extern void cdc_acm_data_send_with_dtr_test(void);
//            cdc_acm_data_send_with_dtr_test();
//        }  
        
    }
}

