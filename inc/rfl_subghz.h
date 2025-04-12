/*
 * rfl_subghz.c
 *
 *  Created on: Mar 30, 2025
 *      Author: patry
 */

#ifndef INC_RFL_SUBGHZ_H_
#define INC_RFL_SUBGHZ_H_

#include "rfl_types.h"

void rfl_subghz_init(void);

void rfl_subghz_init(void);
void rfl_subghz_initTx(void);
void rfl_subghz_initRx(void);
void rfl_subghz_transmit(uint8_t payload[64]);
void rfl_subghz_setRxContinous(void);

void rfl_subghz_setPower(RFTxPower power);

void subghz_clear_irq(void);

void subghz_handle_irq(void);

#endif /* INC_RFL_SUBGHZ_H_ */
