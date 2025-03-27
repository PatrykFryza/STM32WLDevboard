/*
 * rfl_gpio.c
 *
 *  Created on: Mar 27, 2025
 *      Author: Damian Fryza
 */

#include "rfl_gpio.h"
#include "stm32wl55xx.h"

void rfswitch_init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //GPIOA clock enable

	//general purpose output
	GPIOA->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE6);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODE7_Pos) | (0b01 << GPIO_MODER_MODE6_Pos);

	//medium speed
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED6);
	GPIOA->OSPEEDR |= (0b01 << GPIO_OSPEEDR_OSPEED7_Pos) | (0b01 << GPIO_OSPEEDR_OSPEED6_Pos);

	//No pull
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD6);
}

void led_init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //clock enable

	//output
	GPIOA->MODER &= ~(GPIO_MODER_MODE5_1);
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1);
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_0;

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);
}


void led_state(GPIOState gpio_state){
	switch(gpio_state){
	case set:
		GPIOA->BSRR = GPIO_BSRR_BS5;
		break;
	case reset:
		GPIOA->BSRR = GPIO_BSRR_BR5;
		break;
	case toggle:
		GPIOA->ODR ^= GPIO_ODR_OD5;
		break;
	}
}

void RFR_state(GPIOState gpio_state){
	switch(gpio_state){
	case set:
		GPIOA->BSRR = GPIO_BSRR_BS6;
		break;
	case reset:
		GPIOA->BSRR = GPIO_BSRR_BR6;
		break;
	case toggle:
		GPIOA->ODR ^= GPIO_ODR_OD6;
		break;
	}
}

void RFT_state(GPIOState gpio_state){
	switch(gpio_state){
	case set:
		GPIOA->BSRR = GPIO_BSRR_BS7;
		break;
	case reset:
		GPIOA->BSRR = GPIO_BSRR_BR7;
		break;
	case toggle:
		GPIOA->ODR ^= GPIO_ODR_OD7;
		break;
	}
}


