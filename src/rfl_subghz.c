/*
 * rfl_subghz.c
 *
 *  Created on: Mar 28, 2025
 *      Author: patry
 */

#include "stm32wlxx.h"
#include "rfl_clocks.h"
#include "rfl_gpio.h"
#include "stm32wlxx_hal.h"


void rfl_radio_reset(void){
	//radio reset
	RCC->CSR |= RCC_CSR_RFRST;
	HAL_Delay(5);
	RCC->CSR &= ~RCC_CSR_RFRST;
	while(RCC->CSR & RCC_CSR_RFRSTF); //wait for reset flag clear
}


void rfl_radio_init(){
	HSE32_Radio_init();
	rfswitch_init();

	RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN; //enable subghz SPI clock
//
	NVIC_SetPriority(SUBGHZ_Radio_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

	EXTI->IMR2 |= EXTI_IMR2_IM44; //enable IRQ for CPU1
//	EXTI->C2IMR2 |= EXTI_IMR2_IM44; //enable IRQ for CPU2

	rfl_radio_reset();

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //Unselect NSS pin

	PWR->CR3 &= ~PWR_CR3_EWRFBUSY;
	PWR->CR3 |= PWR_CR3_EWRFBUSY; //clear and set radio wakeup signal

	PWR->SCR |= PWR_SCR_CWRFBUSYF;

	uint32_t BaudratePrescaler = SPI_CR1_BR_1;

	/* Enable SUBGHZSPI Peripheral */
	SUBGHZSPI->CR1 &= ~SPI_CR1_SPE;
	SUBGHZSPI->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSI | BaudratePrescaler | SPI_CR1_SSM);
	SUBGHZSPI->CR2 |= (SPI_CR2_FRXTH |  SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);


	SUBGHZSPI->CR1 |= SPI_CR1_SPE;



}
