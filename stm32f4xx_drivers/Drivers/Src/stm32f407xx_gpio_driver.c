/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 5 באוק׳ 2022
 *      Author: Eden
 */

#include "stm32f407xx_gpio_driver.h"


/**
 * @brief Enables the given GPIO's clock
 *
 * @param pGPIOx - base address of the GPIO peripheral
 * @param EnableOrDisable - ENABLE or DISABLE macros
 */
void GPIO_PeripheralClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnableOrDisable)
{
	if(EnableOrDisable == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else // disable
	{
		if(pGPIOx == GPIOA)
				GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			 	GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			 	GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			 	GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			 	GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			 	GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			 	GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			 	GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)
				GPIOI_PCLK_DI();
	}
}

/* Init */

/**
 * @brief
 * Initializes the GPIO
 * @param pGPIOHandle - GPIO handler's pointer
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp=0; // temp register
	uint32_t temp_cr_number, temp_cr_offset;
	uint32_t alternate_func_location;
	// Configure gpio pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{ // Non-interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing the relevant bits
		pGPIOHandle->pGPIOx->MODER |= temp; // setting the relevant bits
	}
	else
	{ // interrupt modes - TBA

		//Configure GPIO Mode:
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{ //config FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{ //config RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{ //config FSRT+RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure gpio port in STSCFG_EXTICR
		temp_cr_number = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp_cr_offset = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp_cr_number] = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) << (temp_cr_offset);
		//enable etxi interrupt using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp=0;

	// configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing the relevant bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	// configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing the relevant bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	// configute optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing the relevant bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//configure alt func
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{

		alternate_func_location = (4 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8) // modify LOW
		{
			pGPIOHandle->pGPIOx->AFR[0] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << alternate_func_location );
		}
		else //modify HIGH
		{
			pGPIOHandle->pGPIOx->AFR[1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << alternate_func_location );
		}
	}
}

/**
 * @brief
 * Returns the GPIO to reset state
 * @param pGPIOx - base address of the GPIO peripheral
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();
}


/* Data Read/Write */

/**
 * @brief
 * Read the GPIO value for the given pin number
 * @param pGPIOx - base address of the GPIO peripheral
 * @param PinNumber
 * @return 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**
 * @brief
 * Read the entire GPIO value
 * @param pGPIOx - base address of the GPIO peripheral
 * @return
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**
 * @brief
 * Write value to specific GPIO pin
 * @param pGPIOx - base address of the GPIO peripheral
 * @param PinNumber
 * @param Value
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(1 << PinNumber);
	else
		pGPIOx->ODR |= (1 << PinNumber);
}

/**
 * @brief
 * Write value to GPIO port
 * @param pGPIOx - base address of the GPIO peripheral
 * @param Value
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**
 * @brief
 * Toggle GPIO pin number value (from 1 to 0 / from 0 to 1)
 * @param pGPIOx - base address of the GPIO peripheral
 * @param PinNumber
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/* IRQ Configuration and handling */

/**
 * @brief
 * Enable or Disable IRQs
 * @param IRQNumber
 * @param EnableOrDisable - ENABLE or DISABLE macros
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable)
{
	uint32_t IRQ_Id, IRQ_Offset;
	//choose relevant NVIC Register according to IRQNumber
	IRQ_Id = IRQNumber / 32; // each NVIC registers can control 32 IRQs
	IRQ_Offset = IRQNumber % 32;
	switch(IRQ_Id)
	{
	case 0:
		if(EnableOrDisable == ENABLE)
			*NVIC_ISER0 |= (1 << IRQ_Offset);
		if(EnableOrDisable == DISABLE)
			*NVIC_ICER0 |= (1 << IRQ_Offset);
		break;
	case 1:
		if(EnableOrDisable == ENABLE)
			*NVIC_ISER1 |= (1 << IRQ_Offset);
		if(EnableOrDisable == DISABLE)
			*NVIC_ICER1 |= (1 << IRQ_Offset);
		break;
	case 2:
		if(EnableOrDisable == ENABLE)
			*NVIC_ISER2 |= (1 << IRQ_Offset);
		if(EnableOrDisable == DISABLE)
			*NVIC_ICER2 |= (1 << IRQ_Offset);
		break;
	case 3:
		if(EnableOrDisable == ENABLE)
			*NVIC_ISER3 |= (1 << IRQ_Offset);
		if(EnableOrDisable == DISABLE)
			*NVIC_ICER3 |= (1 << IRQ_Offset);
		break;
	default:
		break;
	}
}

/// @brief
/// Sets a priority to a given IRQ number
/// @param IRQNumber
/// @param IRQPriority - number between 0 to 15, 0 is highest priority
void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	//avoid priorities that are above 15:
	if(IRQPriority > 15) IRQPriority = 15;
	uint8_t IRQ_RegisterAddrOffset,IRQ_RegisterOffset;

	IRQ_RegisterAddrOffset = (IRQNumber / 4); // div by 4 to get register number, no need to mult because NVIC_PR_BASE_ADDR is u32
	IRQ_RegisterOffset = (IRQNumber % 4) * 8 + (8 - NO_PR_BITS_IMPLEMENTED); // MOD 4 to get location inside specific register, and MULT 8 because each priority location is 8bits
	//(8 - NO_PR_BITS_IMPLEMENTED) : OFFSET to get the 4 bits that are implemented and modify them.

	* (NVIC_PR_BASE_ADDR + IRQ_RegisterAddrOffset) |= (IRQPriority << IRQ_RegisterOffset);
}

/**
 * @brief
 * IRQ Handling method
 * @param PinNumber
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti pr registers
	if(EXTI->PR & (1 << PinNumber))
		EXTI->PR |= (1 << PinNumber);
}

