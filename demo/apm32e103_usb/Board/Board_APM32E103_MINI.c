/*!
 * @file        Board_APM32E103_MINI.c
 *
 * @brief       This file provides firmware functions to manage Leds and key buttons
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */

#include "Board_APM32E103_MINI.h"

/** @defgroup APM32E103_MINI_Private_Variables
  * @{
  */
GPIO_T* GPIO_PORT[LEDn] = {LED2_GPIO_PORT, LED3_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED2_PIN, LED3_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED2_GPIO_CLK, LED3_GPIO_CLK};

GPIO_T* BUTTON_PORT[BUTTONn] = {KEY1_BUTTON_GPIO_PORT, KEY2_BUTTON_GPIO_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {KEY1_BUTTON_PIN, KEY2_BUTTON_PIN};

const uint32_t BUTTON_CLK[BUTTONn] = {KEY1_BUTTON_GPIO_CLK, KEY2_BUTTON_GPIO_CLK};

const uint16_t BUTTON_EINT_LINE[BUTTONn] = {KEY1_BUTTON_EINT_LINE,KEY2_BUTTON_EINT_LINE};

const GPIO_PORT_SOURCE_T BUTTON_PORT_SOURCE[BUTTONn] = {KEY1_BUTTON_EINT_PORT_SOURCE,KEY2_BUTTON_EINT_PORT_SOURCE};

const GPIO_PIN_SOURCE_T BUTTON_PIN_SOURCE[BUTTONn] = {KEY1_BUTTON_EINT_PIN_SOURCE,KEY2_BUTTON_EINT_PIN_SOURCE};

const IRQn_Type BUTTON_IRQn[BUTTONn] = {KEY1_BUTTON_EINT_IRQn, KEY2_BUTTON_EINT_IRQn};

USART_T* COM_USART[COMn] = {MINI_COM1, MINI_COM2};

GPIO_T* COM_TX_PORT[COMn] = {MINI_COM1_TX_GPIO_PORT, MINI_COM2_TX_GPIO_PORT};

GPIO_T* COM_RX_PORT[COMn] = {MINI_COM1_RX_GPIO_PORT, MINI_COM2_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {MINI_COM1_CLK, MINI_COM2_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {MINI_COM1_TX_GPIO_CLK, MINI_COM2_TX_GPIO_CLK};

const uint32_t COM_RX_PORT_CLK[COMn] = {MINI_COM1_RX_GPIO_CLK, MINI_COM2_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {MINI_COM1_TX_PIN, MINI_COM2_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {MINI_COM1_RX_PIN, MINI_COM2_RX_PIN};

/**
  * @}
  */

/** @defgroup APM32E103_MINI_Private_Functions
  * @{
  */

/*!
 * @brief       Configures LED GPIO.
 *
 * @param       Led: Specifies the Led to be configured.
 *              This parameter can be one of following parameters:
 *              @arg LED2
 *              @arg LED3
 *
 * @retval      None
 */
void APM_MINI_LEDInit(Led_TypeDef Led)
{
    GPIO_Config_T  configStruct;

    /* Enable the GPIO_LED Clock */
    RCM_EnableAPB2PeriphClock(GPIO_CLK[Led]);

    /* Configure the GPIO_LED pin */
    configStruct.pin = GPIO_PIN[Led];
    configStruct.mode = GPIO_MODE_OUT_PP;
    configStruct.speed = GPIO_SPEED_50MHz;

    GPIO_Config(GPIO_PORT[Led], &configStruct);
    GPIO_PORT[Led]->BC = GPIO_PIN[Led];
}

/*!
 * @brief       Turns selected LED On.
 *
 * @param       Led: Specifies the Led to be configured.
 *              This parameter can be one of following parameters:
 *              @arg LED2
 *              @arg LED3
 *
 * @retval      None
 */
void APM_MINI_LEDOn(Led_TypeDef Led)
{
    GPIO_PORT[Led]->BC = GPIO_PIN[Led];
}

/*!
 * @brief       Turns selected LED Off.
 *
 * @param       Led: Specifies the Led to be configured.
 *              This parameter can be one of following parameters:
 *              @arg LED2
 *              @arg LED3
 *
 * @retval      None
 */
void APM_MINI_LEDOff(Led_TypeDef Led)
{
    GPIO_PORT[Led]->BSC = GPIO_PIN[Led];
}

/*!
 * @brief       Toggles the selected LED.
 *
 * @param       Led: Specifies the Led to be configured.
 *              This parameter can be one of following parameters:
 *              @arg LED2
 *              @arg LED3
 *
 * @retval      None
 */
void APM_MINI_LEDToggle(Led_TypeDef Led)
{
    GPIO_PORT[Led]->ODATA ^= GPIO_PIN[Led];
}

/*!
 * @brief       Configures Button GPIO and EINT Line.
 *
 * @param       Button: Specifies the Button to be configured.
 *              This parameter can be one of following parameters:
 *              @arg BUTTON_KEY1: Key1 Push Button
 *              @arg BUTTON_KEY2: Key2 Push Button
 * @param       Button_Mode: Specifies Button mode.
 *              This parameter can be one of following parameters:
 *              @arg BUTTON_MODE_GPIO: Button will be used as simple IO
 *              @arg BUTTON_MODE_EINT: Button will be connected to EINT line
 *                   with interrupt generation capability
 *
 * @retval      None
 */
void APM_MINI_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
    GPIO_Config_T     GPIO_configStruct;
    EINT_Config_T     EINT_configStruct;

    /* Enable the BUTTON Clock */
    RCM_EnableAPB2PeriphClock(BUTTON_CLK[Button] | RCM_APB2_PERIPH_AFIO);

    /* Configure Button pin as input floating */
    GPIO_configStruct.mode = GPIO_MODE_IN_PU;
    GPIO_configStruct.pin = BUTTON_PIN[Button];
    GPIO_Config(BUTTON_PORT[Button], &GPIO_configStruct);

    if (Button_Mode == BUTTON_MODE_EINT)
    {
        /* Connect Button EINT Line to Button GPIO Pin */
        GPIO_ConfigEINTLine(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

        /* Configure Button EINT line */
        EINT_configStruct.line = BUTTON_EINT_LINE[Button];
        EINT_configStruct.mode = EINT_MODE_INTERRUPT;
        EINT_configStruct.trigger = EINT_TRIGGER_FALLING;
        EINT_configStruct.lineCmd = ENABLE;
        EINT_Config(&EINT_configStruct);

        /* Enable and set Button EINT Interrupt to the lowest priority */
        NVIC_EnableIRQRequest(BUTTON_IRQn[Button], 0x0f, 0x0f);
    }
}

/*!
 * @brief       Returns the selected Button state.
 *
 * @param       Button: Specifies the Button to be configured.
 *              This parameter can be one of following parameters:
 *              @arg BUTTON_KEY1: Key1 Push Button
 *              @arg BUTTON_KEY2: Key2 Push Button
 *
 * @retval      The Button GPIO pin value.
 */
uint32_t APM_MINI_PBGetState(Button_TypeDef Button)
{
    return GPIO_ReadInputBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/*!
 * @brief       Configures COM port.
 *
 * @param       COM: Specifies the COM port to be configured.
 *              This parameter can be one of following parameters:
 *              @arg COM1
 *              @arg COM2
 *
 * @retval      None
 */
void APM_MINI_COMInit(COM_TypeDef COM, USART_Config_T* configStruct)
{
    GPIO_Config_T GPIO_configStruct;

    /* Enable GPIO clock */
    RCM_EnableAPB2PeriphClock(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM]);

    if (COM == COM1)
    {
        RCM_EnableAPB2PeriphClock(COM_USART_CLK[COM]);
    }
    else
    {
        RCM_EnableAPB1PeriphClock(COM_USART_CLK[COM]);
    }

    /* Configure USART Tx as alternate function push-pull */
    GPIO_configStruct.mode = GPIO_MODE_AF_PP;
    GPIO_configStruct.pin = COM_TX_PIN[COM];
    GPIO_configStruct.speed = GPIO_SPEED_50MHz;
    GPIO_Config(COM_TX_PORT[COM], &GPIO_configStruct);

    /* Configure USART Rx as input floating */
    GPIO_configStruct.mode = GPIO_MODE_IN_FLOATING;
    GPIO_configStruct.pin = COM_RX_PIN[COM];
    GPIO_Config(COM_RX_PORT[COM], &GPIO_configStruct);

    /* USART configuration */
    USART_Config(COM_USART[COM], configStruct);

    /* Enable USART */
    USART_Enable(COM_USART[COM]);
}

/**
  * @}
  */


