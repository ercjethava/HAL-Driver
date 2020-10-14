/*
 *  uart_hal_driver.c
 *	Implementation of API used by application to control USART peripheral
 *  Created on: Oct 10, 2020
 *  Author: Chirag Jethava
 */

#include "usart_hal_driver.h"

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Use to Set BaudRate for UART
 *
 * @param[in]         - UART Register base address
 * @param[in]         - BaudRate
 * 
 * @return            - None
 *
 * @Note              -

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}
	else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		//over sampling by 16
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_Initialize
 *
 * @brief             - This function is used to initialize UART port
 *
 * @param[in]         - UART handle
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_Initialize(USART_Handle_t *pUSARTHandle)
{

	uint32_t tempreg = 0;

	/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//Configuration of parity control bit fields
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - Use to enables or disables UART peripheral
 *
 * @param[in]         - UART register
 * @param[in]         - Enable/ Disable
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t control)
{
	if (control == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Use to control UART Peripheral clock
 *
 * @param[in]         - UART Register
 * @param[in]         - Enable/Disable
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t control)
{
	if (control == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PCCK_DI();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PCCK_DI();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PCCK_DI();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_DI();
		}
	}
}
/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Use to get USART Status flag
 *
 * @param[in]         - USART Register
 * @param[in]         - SttausFlagName
 *
 * @return            - SET/Reset
 *
 * @Note              -

 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if (pUSARTx->SR & StatusFlagName)
	{
		return SET;
	}

	return RESET;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Use to send data to USART
 *
 * @param[in]         - USART Handle
 * @param[in]         - Transmit buffer
 * @param[in]         - Length of transmitting message
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;

	//Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE))
			;

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC))
		;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Use to receive data on USART
 *
 * @param[in]         - USART Handle
 * @param[in]         - Received buffer
 * @param[in]         - Length of received message
 *
 * @return            - None
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		// Wait until RXNE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// receive 9bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used so all 9bits will be of user data

				// read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				// Increment the pRxBuffer
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity is used\ so 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			// receive 8bit data in a frame

			// check are we using USART_ParityControl control or not
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used so all 8bits will be of user data

				// read 8 bits from DR
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				// Parity is used, so 7 bits will be of user data and 1 bit is parity

				// read only 7 bits so masking the DR with 0X7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             - Use to send data to USART in interrupot mode
 *
 * @param[in]         - USART Handle
 * @param[in]         - Trasmit buffer
 * @param[in]         - Length of trasmitting message
 *
 * @return            - USART Trasmit Status 
 *
 * @Note              -

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataWithIT
 *
 * @brief             - Use to received data on USART in interrupt mode
 *
 * @param[in]         - USART Handle
 * @param[in]         - Received buffer
 * @param[in]         - Length of received message
 *
 * @return            - USART Receive Status 
 *
 * @Note              -

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		// enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - Use to clear USART Status Register flag
 *
 * @param[in]         - USART register
 * @param[in]         - Status flag name
 *
 * @return            - None
 *
 * @Note              - 
 *

 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - Use to configure USART Interrupt
 *
 * @param[in]         - Interrupt number
 * @param[in]         - Enable/Disable
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t control)
{

	if (control == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 6 && IRQNumber < 96)
		{
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - Use to configure USART Interrupt Priority
 *
 * @param[in]         - Interrupt number
 * @param[in]         - Interrupt Priority
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// calculate IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - USART Interrupt handler
 *
 * @param[in]         - USART Handle
 *
 * @return            - None
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	uint32_t temp3 = 0;

	uint16_t *pdata = NULL;

	/*		Check for TC flag    */

	// check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	// code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if (temp1 && temp2)
	{
		// interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if (!pUSARTHandle->TxLen)
			{
				// clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// clear the TCIE control bit

				// Reset application state
				pUSARTHandle->TxBusyState = USART_READY;

				// Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// Reset length to zero
				pUSARTHandle->TxLen = 0;

				// Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*  Check for TXE flag */

	// check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	// check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2)
	{
		// interrupt is because of TXE

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Keep sending data until Txlen reaches to zero
			if (pUSARTHandle->TxLen > 0)
			{
				// Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t *)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					// check for USART_ParityControl
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used in this transfer , so 9bits of user data will be sent
						// increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						// Parity bit is used in this transfer so 8bits of user data will be sent
						// 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				}
				else
				{
					// 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					// Increment buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}
			}
			if (pUSARTHandle->TxLen == 0)
			{
				// TxLen is zero
				// clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/* 		Check for RXNE flag 	*/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2)
	{
		// interrupt is because of RXNE
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if (pUSARTHandle->RxLen > 0)
			{
				// Check the USART_Word Length to check whether receiving 9bit or 8 bit data in frame
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// receiving 9bit data in a frame

					// check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used so all 9bits will be of user data

						// read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t *)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

						// increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					// receive 8bit data in a frame

					// check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used so all 8bits will be of user data

						// read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}

					else
					{
						// Parity is used so 7 bits will be of user data and 1 bit is parity

						// read only 7 bits so mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}

					// Increment pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}

			} //if of >0

			if (!pUSARTHandle->RxLen)
			{
				// Disable RXNE
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/*		Check for CTS flag 		*/
	//Note : CTS feature is not applicable for UART4 and UART5

	// check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	// check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	// check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp1 && temp2)
	{
		// Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		// Interrupt is because of CTS
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	/*		Check for IDLE detection flag 		*/

	// check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	// check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if (temp1 && temp2)
	{
		// Clear the IDLE flag
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// Interrupt is because of idle condition
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/*		Check for Overrun detection flag 	*/

	// check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	// check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (temp1 && temp2)
	{
		//Need not to clear the ORE flag instead support API for the application to clear the ORE flag
		// Because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/*		Check for Error Flag 		*/

	// Noise Flag, Overrun error and Framing Error in MultiBuffer communication

	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if (temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if (temp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if (temp1 & (1 << USART_SR_NE))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if (temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - This function is used to handle event occurred in USART ISR
 *						
 * @param[in]         - USARt handle 
 * @param[in]         - Event type
 *
 * @return            - None
 *
 * @Note              - This function is override by Application to capture event

 */

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event_type)
{
	// This is a weak implementation
	// User application can override this function
}
