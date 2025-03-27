/*
 * rfl_clocks.h
 *
 *  Created on: Mar 3, 2025
 *      Author: Damian Fryza
 */

#ifndef INC_STM32WL_CLOCKS_H_
#define INC_STM32WL_CLOCKS_H_

void HSI16_init();

void HSI_PLL48_init();

void HSE32_Radio_init();
void HSE32_CPU_init();

#endif /* INC_STM32WL_CLOCKS_H_ */
