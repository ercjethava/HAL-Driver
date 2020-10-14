# HAL driver development for ARM CORTEX MCU
In this project I developed Hardware Abtraction Driver for the different peripheral devices like: USART, I2C , SPI, GPIO from Scratch with the help of perticualr MCU Datasheet. In this project I used STM32F446RE MCU from STMicroelectronics as a reference MCU. So in order to use this HAL Driver you have to modify stm32f446xx.h header file located into HAL-Driver/HAL_Driver/Inc based on your MCU as this file contains Processor Specific Details.

## Sections
- [HAL driver development for ARM CORTEX MCU](#hal-driver-development-for-arm-cortex-mcu)
  - [Sections](#sections)
  - [GPIO](#gpio)
  - [I2C](#i2c)
  - [SPI](#spi)
  - [USART](#usart)
  - [File Structure:](#file-structure)
  - [How to Use](#how-to-use)

---
## GPIO
This driver is used to control GPIO PIN functionality.

## I2C
This driver is used to control I2C functionality. It provides maximum speed upto 3.4 Mbps.This interface can be use to communicate between devices such as sensors,EEPROM, mictocontrolle etc where we do not required higher speed. It is Half-Duplex.

## SPI
This driver is used to control SPI functionality. This interface provides high speed data transfer. So it is use for the device such as LED screen , SD Card where high speed is required to read or write. It is Full-Duplex.

## USART
This driver is used to control USART functionality. It is a type of Asyncronous communication protocol. It is Full-Duplex.

## File Structure:
All source code related to HAL Driver is located inside HAL_Driver directory.

HAL-Driver/HAL_Driver/Src  -> This directory includes implementation of HAL Driver API

HAL-Driver/HAL_Driver/Inc -> This directory includes declaration of HAL Driver API. In order to use functionality of perticular Peripheral Device application code have to include specific HAL device driver header file into their Application code.

HAL-Driver/Src -> This directory contains Sample Appliction Firmware code which shows usage of HAL Driver API. 

## How to Use

In order to use this HAL Driver in your application firmware follow following steps:

Step1: Copy HAL_Driver directory in your Project_Directory.

Step2: Include above copied directory (Project_Directory/HAL_Driver/Inc) into your compilation tool chain.

Step3: Include specific Peripheral HAL driver header file into your appliaction firmware. Check sample Application Code located into HAL-Driver/Src for the understanding of HAL Driver usage.

Stem4: Build your project.