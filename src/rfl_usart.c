/*
 * rfl_usart.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Damian Fryza
 */

#include "rfl_usart.h"
#include "stm32wl55xx.h"

#define BAUD_RATE (SystemCoreClock/115200)

void USART1_Init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //GPIOB clock enable

    GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7); //clear GPIOB pin 6 and  7 mode
    GPIOB->MODER |= (0b10 << GPIO_MODER_MODE6_Pos) | (0b10 << GPIO_MODER_MODE7_Pos); //set GPIOB pin 7 and 8

    //as alternate function
    GPIOB->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL6_Pos); //alternate function 7 for PB6
    GPIOB->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL7_Pos); //alternate function 7 for PB7 (table in DS13293 page 62)

    RCC->CCIPR |= (0b01 << RCC_CCIPR_USART1SEL_Pos); //usart1 SYSCLK
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //usart1 clock enable for cpu1

    USART1->BRR = BAUD_RATE;

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; //transmit and receive enable
    USART1->CR1 |= USART_CR1_UE; //usart enable
}

int __io_putchar(int ch){
    while (!(USART1->ISR & USART_ISR_TXE_TXFNF));
    USART1->TDR = ch;
    return ch;
}

int _write(int fd, char* ptr, int len){
    for (int i = 0; i < len; i++) {
        __io_putchar(ptr[i]);
    }
    return len;
}
