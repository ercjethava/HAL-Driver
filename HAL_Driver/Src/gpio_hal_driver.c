/*
 * 	gpio_hal_driver.c
 *  Implementation of API used by application to control GPIO 
 *  Created on: Oct 10, 2020
 *  Author: Chirag Jethava
 */

#include "gpio_hal_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - GPIO Register base address
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnDi)
{
    if (EnDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
    }
}

/*********************************************************************
 * @fn      		  - GPIO_Initialize
 *
 * @brief             - This function is used to initialize GPIO such as mode,speed,output type,intrupt
 *
 * @param[in]         - GPIO Handle pointer
 *
 * @return            - None
 *
 * @Note              - 

 */
void GPIO_Initialize(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    //1 . configure GPIO PIN MODE

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        //interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // Configure the FTSR
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //Clear the corresponding RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            //Configure the RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //Clear the corresponding RTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            //Configure both FTSR and RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            //Clear the corresponding RTSR bit
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        //Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        //Enable EXTI interrupt delivery using IMR
        EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    }

    //2. Configure speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    //3. Configure the  pull up/pull down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    //4. Configure Output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    //5. Configure alternative  functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        //configure the alternative function registers.
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function is used to Reset GPIO peripheral register 
 *
 * @param[in]         - GPIO register base address
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_DeInitialize(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Used to read data from particular PIN number of GPIO port
 *
 * @param[in]         - GPIO Register base address
 * @param[in]         - GPIO pin number
 *
 * @return            - 0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;

    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

    return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Used to read entire GPIO port data
 *
 * @param[in]         - GPIO Register base address
 *
 * @return            - 16 bit data 
 *
 * @Note              -

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;

    return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Used to write 0 or 1 to GPIO port PIN
 *
 * @param[in]         - GPIO Register base address
 * @param[in]         - Pin number
 * @param[in]         - 1 or 0
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

    if (Value == GPIO_PIN_SET)
    {
        //write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        //write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Used to write data to GPIO Port
 *
 * @param[in]         - GPIO Register base address
 * @param[in]         - Value to be write into GPIO port
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Used to Toggle GPIO PIN
 *
 * @param[in]         - GPIO Register base address
 * @param[in]         - GPIO Port PIN Number
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Used for GPIO Interrupt configuration
 *
 * @param[in]         - Interrupt number
 * @param[in]         - Control macro Enable or Disable
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t control)
{

    if (control == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            //program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
        {
            //program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ISER2 register //64 to 95
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            //program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            //program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ICER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Used to set Interrupt Priority for GPIO Interrupt
 *
 * @param[in]         - Interrupt number
 * @param[in]         - Interrupt priority
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // Calculating Interupt Priority register for a specific interrupt number
    uint8_t ipr = IRQNumber / 4;
    uint8_t ipr_section = IRQNumber % 4;

    uint8_t shift = (8 * ipr_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + ipr) |= (IRQPriority << shift);
}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Used to clear pending register related to particular PIN when interrupt occurs
 *
 * @param[in]         - GPIO PIN number
 *
 * @return            - None
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    //check if Pending Register bit is Set related to GPIO PIN number
    if (EXTI->PR & (1 << PinNumber))
    {
        // check if Pending Register bit
        EXTI->PR |= (1 << PinNumber);
    }
}
