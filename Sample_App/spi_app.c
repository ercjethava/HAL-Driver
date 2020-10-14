/*
 *  spi_app.c
 *  Sample application shows SPI HAL driver usage
 *  Created on: Oct 11, 2020
 *  Author: Chirag Jethava
 *  email:  ercjethava@gmail.com
 */


#include "string.h"
#include "stdint.h"
#include "stm32f446xx.h"
#include "gpio_hal_driver.h"
#include "spi_hal_driver.h"


/* 	
*	SPI PIN Configuaraton
* 	PB14	-> 	SPI2_MISO
* 	PB15	-> 	SPI2_MOSI
* 	PB13 	-> 	SPI2_SCLK
* 	PB12 	-> 	SPI2_NSS
* 	ALT function mode : 5
*/



void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Initialize(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Initialize(&SPIPins);

	//MISO  -> Not needed to configure MISO as Master is not receiving anything
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Initialize(&SPIPins);

	//NSS-> for Hardware slave management
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Initialize(&SPIPins);


}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; 					

	SPI_Initialize(&SPI2handle);
}

int main(void)
{
	char trasmit_data[] = "Sending Data to Arduino via SPI";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 Peripheral
	SPI2_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	// Enable SPI2 Peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	// Master first sends number of bytes information  to slave
	uint8_t length = strlen(trasmit_data);
	SPI_SendData(SPI2,&length,1);

	// Then Master sends whole message
	SPI_SendData(SPI2,(uint8_t*)trasmit_data,length);

	// check SPI is busy or not
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	// Disable SPI2 Peripheral after completion of trasnmission 
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;

}
