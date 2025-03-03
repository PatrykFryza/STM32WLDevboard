/*
 * STM32WL_clocks.c
 *
 *  Created on: Mar 3, 2025
 *      Author: Akkarin
 */

#include "STM32WL_clocks.h"
#include "stm32wlxx.h"
#include "stm32wl55xx.h"

void HSI16_init(){
	RCC->CR |= RCC_CR_HSION; // HSI enable

	while((RCC->CR & RCC_CR_HSIRDY) != 0); // wait for ready
	RCC->CFGR &= ~RCC_CFGR_SW; // clear SW[0:1]
	RCC->CFGR |= RCC_CFGR_SW_0; // set HSI16 as system clock
}

void HSE32_CPU_init(){
	RCC->CR |= RCC_CR_HSEBYPPWR; // PB0 as Vddtcxo
	RCC->CR |= RCC_CR_HSEON; // HSE32 for CPU enabled

	while((RCC->CR & RCC_CR_HSERDY) != 0); // wait for ready
	RCC->CFGR &= ~RCC_CFGR_SW; // clear SW[0:1]
	RCC->CFGR |= RCC_CFGR_SW_1; // set HSE32 as system clock
}
