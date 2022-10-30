/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: 30 באוק׳ 2022
 *      Author: Eden
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;		// master or slave
	uint8_t SPI_BusConfig;	  	// full-duplex, half-duplex, simplex
	uint8_t SPI_SclkSpeed; 		// baude rate
	uint8_t SPI_DFF;			// data frame format
	uint8_t SPI_CPOL;			// clock polarity
	uint8_t SPI_CPHA;			// clock phase
	uint8_t SPI_SSM;			// software slave management
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t* pSPIx; // holds the base address of SPIx(x=1,2,3)
	SPI_Config_t SPI_Config;
}SPI_Handle_t;

/* SPI_DeviceMode */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/* SPI_BusConfig */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

/* SPI_SclkSpeed */
#define SPI_SCLK_SPEED_DIV2					0 // sets serial clock to Fclk / 2 -> fastest baude rate
#define SPI_SCLK_SPEED_DIV4					1 // sets serial clock to Fclk / 4
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/* SPI_DFF - data frame format: 8 or 16 bits */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1


// CPOL and CPHA selects the data capture clock edge

/* SPI_CPOL - Clock polarity */
#define SPI_CPOL_LOW						0	// CK to 0 when idle
#define SPI_CPOL_HIGH						1	// CK to 1 when idle

/* SPI_CPHA - Clock phase */
#define SPI_CPHA_LOW						0	// first data capture edge
#define SPI_CPHA_HIGH						1	// 2nd data capture edge

/* SPI_SSM - Software slave management */
#define SPI_SSM_EN							1
#define SPI_SSM_DE							0



/**************************************************************************
 * 					 Bit fields macros of SPI Registers					  *
 **************************************************************************/

/* Bitfields for SPI CR1 */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSB_FIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRC_NEXT					12
#define SPI_CR1_CRC_EN						13
#define SPI_CR1_BIDI_OE						14
#define SPI_CR1_BIDIMODE					15

/* Bitfields for SPI CR2 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXNEIE						7

/* Bitfields for SPI status register */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRC_ERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8


/**************************************************************************
 * 					 APIs supported by this driver						  *
 **************************************************************************/
/* Peripheral clock setup */
void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, uint8_t EnableOrDisable);

/* Init and De-Init*/
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/* Data send and receive */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQHandling(SPI_Handle_t* pHandle);
/* IRQ Config and ISR Handling */


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
