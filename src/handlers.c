/*
 * handlers.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Damian Fryza
 */

#include "handlers.h"

#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "rfl_subghz.h"

void SUBGHZ_Radio_IRQHandler(void){
	subghz_clear_irq();
	subghz_handle_irq();
}

void SysTick_Handler(void){
//	HAL_IncTick();
}

void NMI_Handler(void){
	while(1){
		__NOP();
	}
}

void HardFault_Handler(void){
	while(1){
		__NOP();
	}
}

void MemManage_Handler(void){
	while(1){
		__NOP();
	}
}

void BusFault_Handler(void){
	while(1){
		__NOP();
	}
}

void UsageFault_Handler(void){
	while(1){
		__NOP();
	}
}

void SVC_Handler(void){
	__NOP();
}

void DebugMon_Handler(void){
	__NOP();
}

void PendSV_Handler(void){
	__NOP();
}
