/*
 * hal_based.subghz.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Damian Fryza
 */

#ifndef INC_HAL_BASED_SUBGHZ_H_
#define INC_HAL_BASED_SUBGHZ_H_

void subghz_init(void);

void subghz_init(void);
void subghz_init_tx(void);
void subghz_init_rx(void);

void subghz_clear_irq(void);

void subghz_handle_irq(void);

//void subghz_rx_on(void);
//void subghz_rx_off(void);

#endif /* INC_HAL_BASED_SUBGHZ_H_ */
