/*
 *  i2c_app.c
 *  Sample application shows I2C HAL driver usage
 *  Created on: Oct 11, 2020
 *  Author: Chirag Jethava
 *  email:  ercjethava@gmail.com
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "gpio_hal_driver.h"
#include "i2c_hal_driver.h"

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

#define BUTTON_PIN GPIO_PIN_NO_13
#define BUTTON_GPIO_PORT GPIOC
#define LED_PIN GPIO_PIN_NO_5
#define LED_GPIO_PORT GPIOA

uint8_t rxComplete = RESET;

// I2C Handle
I2C_Handle_t I2C1Handle;

//Receive buffer
uint8_t receive_buff[32];



/*
*  I2C PIN Configuaraton
*
*  PB6  ->  SCL
*  PB7  ->  SDA
*/

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCL PIn configuration
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Initialize(&I2CPins);

    // SDL PIn configuration
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Initialize(&I2CPins);
}

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Initialize(&I2C1Handle);
}

void GPIO_ButtonAndLedInit(void)
{
    GPIO_Handle_t GPIOBtn, GPIOLed;

    // BUTTON GPIO configuration
    GPIOBtn.pGPIOx = BUTTON_GPIO_PORT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Initialize(&GPIOBtn);

    // LED GPIO configuration
    GPIOLed.pGPIOx = LED_GPIO_PORT;
    GPIOLed.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
    GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Initialize(&GPIOLed);
}

int main(void)
{

    uint8_t commandcode = 0;

    uint8_t length = 0;

    GPIO_ButtonAndLedInit();

    // I2C PIN init
    I2C1_GPIOInits();

    // I2C Peripheral configuration
    I2C1_Inits();

    // I2C IRQ configuration
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    // Enable I2C Peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // ACK bit is made 1 after PE=1
    I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

    while (1)
    {
        //wait till button is pressed
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        commandcode = 0x10;

        // Master send 0x10 Command to Slave to request number of bytes
        while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        // AT receiving 0x10 command slave respode number of bytes it is going to send
        while (I2C_MasterReceiveDataIT(&I2C1Handle, &length, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

        commandcode = 0x20;
        // Master send 0x20 command to request number of bytes from slave device
        while (I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
        
        // At Receiving 0x10 command slave responding length of bytes to Master device
        while (I2C_MasterReceiveDataIT(&I2C1Handle, receive_buff, length, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

        rxComplete = RESET;

        //wait till Receive data completes
        while (rxComplete != SET);

        receive_buff[length + 1] = '\0';

        rxComplete = RESET;
    }

    return 0;
}

// Event Interrupt handler
void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle);
}

// Error Interrupt handler
void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C1Handle);
}

// Application Event callback funtion to capture Event received from ISR
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    if (AppEv == I2C_EV_TX_CMPLT)
    {
       // Trasmission is completed
    }
    else if (AppEv == I2C_EV_RX_CMPLT)
    {
        // Receiving data is completed
        rxComplete = SET;
    }
    else if (AppEv == I2C_ERROR_AF)
    {
        // Error : Ack failure
        // Master ack failure happens when slave fails to send ack for the byte
        //sent from the master

        // Close I2C trasnmission
        I2C_CloseSendData(pI2CHandle);

        // Generate the stop condition to release the bus
        I2C_GenerateStopCondition(I2C1);

        while (1);
    }
}
