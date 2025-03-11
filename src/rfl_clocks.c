/*
 * STM32WL_clocks.c
 *
 *  Created on: Mar 3, 2025
 *      Author: Akkarin
 */

#include "rfl_clocks.h"
#include "stm32wl55xx.h"

void HSI16_init(){
	RCC->CR |= RCC_CR_HSION; // HSI enable

	while(!(RCC->CR & RCC_CR_HSIRDY)); // wait for ready
	RCC->CFGR &= ~RCC_CFGR_SW; // clear SW[0:1]
	RCC->CFGR |= (0b01 << RCC_CFGR_SW_Pos); // set HSI16 as system clock

	SystemCoreClockUpdate();
}

void HSI_PLL48_init(void){
	PWR->CR1 &= ~PWR_CR1_VOS; // reset voltage scaling range selection
	PWR->CR1 |= (0b01 << PWR_CR1_VOS_Pos); // set Range 1 scaling

	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= (0b010 << FLASH_ACR_LATENCY_Pos); // 2 wait flash latency

	RCC->CR |= RCC_CR_HSION; // HSI enable
	while(!(RCC->CR & RCC_CR_HSIRDY)); // wait for HSIRDY becomes 1

	RCC->CR &= ~RCC_CR_PLLON; //turn off pll
	while(RCC->CR & RCC_CR_PLLRDY); // wait for PLLRDY unlocked - 0

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; // Reset PLL source
	RCC->PLLCFGR |= (0b10 << RCC_PLLCFGR_PLLSRC_Pos); // Set HSI16 as PLL source

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM; // Reset PLLM division
	RCC->PLLCFGR |= (0b000 << RCC_PLLCFGR_PLLM_Pos); // main division factor for PLLs /1

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN; // Reset PLL main multiplication
	RCC->PLLCFGR |= (0b110 << RCC_PLLCFGR_PLLN_Pos); // Set x6 PLL main multiplication

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR; // Reset PLLR division
	RCC->PLLCFGR |= (0b001 << RCC_PLLCFGR_PLLR_Pos); // Set /2 PLL main division

	RCC->CR |= RCC_CR_PLLON; // turn on PLL

	while(!(RCC->CR & RCC_CR_PLLRDY)); //wait for PLL ready (PLLRDY is 1)

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; //turn on PLLCLK output

	RCC->CFGR &= ~RCC_CFGR_SW; // reset system clock
	RCC->CFGR |= (0b11 << RCC_CFGR_SW_Pos); // set PLLR as system clock

	while(((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos) != 0b11); //wait until SWS is 11 (PLLR as system clock)

	SystemCoreClockUpdate();
}

void HSE32_CPU_init(){
	RCC->CR |= RCC_CR_HSEBYPPWR; // PB0 as Vddtcxo
	RCC->CR |= RCC_CR_HSEON; // HSE32 for CPU enabled

	while((RCC->CR & RCC_CR_HSERDY) != 0); // wait for ready
	RCC->CFGR &= ~RCC_CFGR_SW; // clear SW[0:1]
	RCC->CFGR |= RCC_CFGR_SW_1; // set HSE32 as system clock
}
