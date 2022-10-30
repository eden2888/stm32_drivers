/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 30 באוק׳ 2022
 *      Author: Eden
 */


#include "stm32f407xx_spi_driver.h"

/// @brief
/// Enable or Disable the given SPI peripheral's clock
/// @param pSPIx
/// @param EnableOrDisable
void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, uint8_t EnableOrDisable)
{

	if(EnableOrDisable == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if(pSPIx == SPI3)
			SPI3_PCLK_EN();
	}
	else // disable
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)
			SPI3_PCLK_DI();
	}
}

/// @brief
/// Initializes the given SPI peripheral
/// @param pSPIHandle
void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	uint32_t temp=0; // temp register for configuration help

	//configure device mode
	if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
		temp |= (1 << SPI_CR1_MSTR);
	else if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE)
		temp &= (0 << SPI_CR1_MSTR);
	else; //handle invalid values - TBA

	//bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
		temp &= (0 << SPI_CR1_BIDIMODE);
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		temp |= (1 << SPI_CR1_BIDIMODE);
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp &= (0 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}
	//sclk speed

	//handle invalid values - for now set to div2
	if(pSPIHandle->SPI_Config.SPI_SclkSpeed < SPI_SCLK_SPEED_DIV2 || pSPIHandle->SPI_Config.SPI_SclkSpeed > SPI_SCLK_SPEED_DIV256)
		pSPIHandle->SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//dff
	//handle invalid values - for now set for 8bit as default:
	if(pSPIHandle->SPI_Config.SPI_DFF != SPI_DFF_8BITS && pSPIHandle->SPI_Config.SPI_DFF != SPI_DFF_16BITS)
		pSPIHandle->SPI_Config.SPI_DFF = SPI_DFF_8BITS;

	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//cpol, cpha
	//handle invalid values
	if (pSPIHandle->SPI_Config.SPI_CPOL != SPI_CPOL_HIGH && pSPIHandle->SPI_Config.SPI_CPOL != SPI_CPOL_LOW)
		pSPIHandle->SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	if (pSPIHandle->SPI_Config.SPI_CPHA != SPI_CPHA_HIGH && pSPIHandle->SPI_Config.SPI_CPHA != SPI_CPHA_LOW)
		pSPIHandle->SPI_Config.SPI_CPHA = SPI_CPHA_LOW;

	temp |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);
	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	//sw slave mgmt
	if (pSPIHandle->SPI_Config.SPI_SSM != SPI_SSM_EN && pSPIHandle->SPI_Config.SPI_SSM != SPI_SSM_DE)
		pSPIHandle->SPI_Config.SPI_SSM = SPI_SSM_DE;

	temp |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);
	pSPIHandle->pSPIx->CR1 = temp;

}

/// @brief
/// Returns the given SPI to reset states
/// @param pSPIx
void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}

/* Data send and receive */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQHandling(SPI_Handle_t* pHandle);
