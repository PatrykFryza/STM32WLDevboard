/*
 * handlers.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Damian Fryza
 */

#ifndef INC_HANDLERS_H_
#define INC_HANDLERS_H_

//Subghz
void SUBGHZ_Radio_IRQHandler(void);

//system handlers
void SysTick_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);

#endif /* INC_HANDLERS_H_ */
