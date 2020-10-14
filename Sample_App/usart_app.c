/*
 *  usart_app.c
 *  Sample application shows USART HAL driver usage
 *  Created on: Oct 11, 2020
 *  Author: Chirag Jethava
 *  email:  ercjethava@gmail.com
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "gpio_hal_driver.h"
#include "usart_hal_driver.h"

/* STM32F446RE Nucleo board pin configuration for Button and LED */
#define B1_Pin GPIO_PIN_NO_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_NO_5
#define LD2_GPIO_Port GPIOA

char uart_msg[] = "Sending Message via UART.....\n\r";

USART_Handle_t Usart2Handle;

void USART2_Init(void)
{
    Usart2Handle.pUSARTx = USART2;
    Usart2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    Usart2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    Usart2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    Usart2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    Usart2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    Usart2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Initialize(&Usart2Handle);
}

void USART2_GPIOInit(void)
{
    GPIO_Handle_t UsartGPIO;

    UsartGPIO.pGPIOx = GPIOA;
    UsartGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    UsartGPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    UsartGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    UsartGPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    UsartGPIO.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    //USART2 TX PIN configure
    UsartGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Initialize(&UsartGPIO);

    //USART2 RX PIN configure
    UsartGPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Initialize(&UsartGPIO);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOButton, GPIOLed;

    // BUTTON GPIO configuration
    GPIOButton.pGPIOx = B1_GPIO_Port;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber = B1_Pin;
    GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Initialize(&GPIOButton);

    // LED GPIO configuration
    GPIOLed.pGPIOx = LD2_GPIO_Port;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = LD2_Pin;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Initialize(&GPIOLed);
}

void delay(void)
{
    for (uint32_t i = 0; i < 100000; i++);
}

int main(void)
{

    GPIO_ButtonInit();

    USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART2, ENABLE);

    while (1)
    {
        // Wait till button pressed
        while (!GPIO_ReadFromInputPin(B1_GPIO_Port, B1_Pin));

        //to avoid button de-bouncing issues
        delay();
        // Send Data on USART2
        USART_SendData(&Usart2Handle, (uint8_t *)uart_msg, strlen(uart_msg));
    }

    return 0;
}
