/*
 * rfl_subghz.c
 *
 *  Created on: Mar 30, 2025
 *      Author: patry
 */

#ifndef INC_RFL_SUBGHZ_H_
#define INC_RFL_SUBGHZ_H_

void subghz_init(void);

void subghz_init(void);
void subghz_init_tx(void);
void subghz_init_rx(void);

void subghz_clear_irq(void);

void subghz_handle_irq(void);

#endif /* INC_RFL_SUBGHZ_H_ */
