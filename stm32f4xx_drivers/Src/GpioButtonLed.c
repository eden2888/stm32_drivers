/*
 * GpioButtonLed.c
 *
 *  Created on: 5 באוק׳ 2022
 *      Author: Eden
 */



#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(uint32_t time)
{
	uint32_t i;
	for(i=0; i<time; i++);
}

int main(void)
{
	//Setup onboard LED at PD12 - green port
	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	//Setup onboard button at PA0
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_PeripheralClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_12);
		delay(250000);
	}
}
