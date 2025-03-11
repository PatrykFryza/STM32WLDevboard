/*
 * rfl_usart.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Damian Fryza
 */

#ifndef INC_RFL_USART_H_
#define INC_RFL_USART_H_

void USART1_Init(void);
int __io_putchar(int ch);
int _write(int fd, char* ptr, int len);

#endif /* INC_RFL_USART_H_ */
