/*
 * rfl_gpio.h
 *
 *  Created on: Mar 27, 2025
 *      Author: Damian Fryza
 */

#ifndef INC_RFL_GPIO_H_
#define INC_RFL_GPIO_H_

typedef enum {
	set,
	reset,
	toggle,
} GPIOState;

void rfswitch_init(void);
void led_init(void);

void led_state(GPIOState gpio_state); //PA5
void RFR_state(GPIOState gpio_state); //PA6
void RFT_state(GPIOState gpio_state); //PA7

#endif /* INC_RFL_GPIO_H_ */
