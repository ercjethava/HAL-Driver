/*
 *  spi_hal_driver.c
 *	Implementation of API used by application to control SPI peripheral
 *  Created on: Oct 10, 2020
 *  Author: Chirag Jethava
 */

#include "spi_hal_driver.h"

/*  private functions */
static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables SPI peripheral clock
 *
 * @param[in]         - SPI Register base address
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t control)
{
	if (control == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Initialize
 *
 * @brief             - This function is used to initialize SPI 
 *
 * @param[in]         - SPI Handle 
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_Initialize(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function is used to get SPI status register flag
 *
 * @param[in]         - SPI base rigister  
 * @param[in]         - SPI status Flag which need to check
 * 
 * @return            - FALG_SET or FLAG_RESET
 *
 * @Note              -

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function is use to send data on MOSI line
 *
 * @param[in]         - SPI base rigister
 * @param[in]         - Trasmit buffer
 * @param[in]         - Trasmit mesage length
 *
 * @return            -  None
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		//2. check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This funciton is used to receive data on SPI MISO line
 *
 * @param[in]         - SPI base rigister
 * @param[in]         - Receive buffer
 * @param[in]         - Received message length
 *
 * @return            - None
 *
 * @Note              - This is blocking call

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET)
			;

		//2. check the DFF bit in CR1
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data from DR to Rxbuffer address
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This fucntion is use to enable or disable SPI peripheral
 *
 * @param[in]         - SPI base rigister
 * @param[in]         - Enable or Disable
 * 
 * @return            -
 *
 * @Note              -

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t control)
{
	if (control == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Use to control NSS singal 
 *
 * @param[in]         - SPI Base rigister
 * @param[in]         - Enable or Disable
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t control)
{
	if (control == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This fucntion is used to contreol SPI interrupt
 *
 * @param[in]         - Interrupt Number 
 * @param[in]         - Enable or Disbale
 * 
 * @return            - None
 *
 * @Note              -

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t control)
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
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
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
		else if (IRQNumber >= 6 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This fucntion is used to contreol SPI interrupt priority
 *
 * @param[in]         - Interrupt Number 
 * @param[in]         - Interrupt priority 
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Calculating Interupt Priority register for a specific interrupt number
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This is a SPI ISR handling routine executed when interupt occured
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_tx_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rx_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*********************************************************************
 * @fn      		  - spi_tx_interrupt_handle
 *
 * @brief             - This is a helper funtion to handle trasnmit interupt
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
static void spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - spi_rx_interrupt_handle
 *
 * @brief             - This is a helper funtion to handle reception interupt
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
static void spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if (pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		//16 bit
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handle
 *
 * @brief             - This is a helper funtion to handle overflow interupt
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*********************************************************************
 * @fn      		  - SPI_CloseTransmisson
 *
 * @brief             - This funciton is used to end trasnmission
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*********************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - This funciton is used to end reception
 *
 * @param[in]         - SPI handle  
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*********************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             - This function is used to clear data and status register
 *
 * @param[in]         - SPI base rigister  
 *
 * @return            - None
 *
 * @Note              -

 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*********************************************************************
 * @fn      		  - SPI_ApplicationEventCallback
 *
 * @brief             - This fucntion is used to handle event occured in SPI ISR
 *						
 * @param[in]         - SPI handle 
 * @param[in]         - Event type
 *
 * @return            - None
 *
 * @Note              - This funtion is override by Application to capture event

 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation
	// User application can override this function
}
