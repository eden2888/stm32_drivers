/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: 30 באוק׳ 2022
 *      Author: Eden
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t* pSPIx; // holds the base address of SPIx(x=1,2,3)
	SPI_Config_t SPI_Config;
}SPI_Handle_t;



/**************************************************************************
 * 					 APIs supported by this driver						  *
 **************************************************************************/
/* Peripheral clock setup */
void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, uint8_t EnableOrDisable);

/* Init and De-Init*/
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/* Data send and receive */

/* IRQ Config and ISR Handling */


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
