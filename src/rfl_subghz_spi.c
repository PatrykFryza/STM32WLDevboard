/*
 * rfl_subghz.c
 *
 *  Created on: Mar 28, 2025
 *      Author: patry
 */

#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "rfl_clocks.h"
#include "rfl_gpio.h"


//#define SUBGHZ_IT_TX_CPLT                   0x0001U
//#define SUBGHZ_IT_RX_CPLT                   0x0002U
//#define SUBGHZ_IT_PREAMBLE_DETECTED         0x0004U
//#define SUBGHZ_IT_SYNCWORD_VALID            0x0008U
//#define SUBGHZ_IT_HEADER_VALID              0x0010U
//#define SUBGHZ_IT_HEADER_ERROR              0x0020U
//#define SUBGHZ_IT_CRC_ERROR                 0x0040U
//#define SUBGHZ_IT_CAD_DONE                  0x0080U
//#define SUBGHZ_IT_CAD_ACTIVITY_DETECTED     0x0100U
//#define SUBGHZ_IT_RX_TX_TIMEOUT             0x0200U
//#define SUBGHZ_IT_LR_FHSS_HOP               0x4000U

#define SUBGHZ_RADIO_WRITE_REGISTER         0x0DU
#define SUBGHZ_RADIO_READ_REGISTER          0x1DU
#define SUBGHZ_RADIO_WRITE_BUFFER           0x0EU
#define SUBGHZ_RADIO_READ_BUFFER            0x1EU

#define SUBGHZ_DUMMY_DATA          0xFFU   /* SUBGHZSPI Dummy Data use for Tx */


void rfl_radio_reset(void){
	//radio reset
	RCC->CSR |= RCC_CSR_RFRST;
	for(uint8_t i = 0; i < 250; ++i);
	RCC->CSR &= ~RCC_CSR_RFRST;
	while(RCC->CSR & RCC_CSR_RFRSTF); //wait for reset flag clear
}


void rfl_radio_spi_init(){
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


static void rfl_SUBGHZSPI_Transmit(uint8_t Data){
  __IO uint32_t count;

  /* Handle Tx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = ((SystemCoreClock*28U)>>19U)*100;

  /* Wait until TXE flag is set */
  do{
    if (count == 0U){
      break;
    }
    count--;
  } while((SUBGHZSPI->SR & SPI_SR_TXE) != (SPI_SR_TXE));

  /* Transmit Data*/
#if defined (__GNUC__)
  __IO uint8_t *spidr = ((__IO uint8_t *)&SUBGHZSPI->DR);
  *spidr = Data;
#else
  *((__IO uint8_t *)&SUBGHZSPI->DR) = Data;
#endif /* __GNUC__ */

  /* Handle Rx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = ((SystemCoreClock*28U)>>19U)*100;

  /* Wait until RXNE flag is set */
  do {
    if (count == 0U){
      break;
    }
    count--;
  } while((SUBGHZSPI->SR & SPI_SR_RXNE) != (SPI_SR_RXNE));

  /* Flush Rx data */
  SUBGHZSPI->DR;
}


static void rfl_SUBGHZSPI_Receive(uint8_t *pData){
  __IO uint32_t count;

  /* Handle Tx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = ((SystemCoreClock*28U)>>19U)*100;

  /* Wait until TXE flag is set */
  do {
    if (count == 0U){
      break;
    }
    count--;
  } while ((SUBGHZSPI->SR & SPI_SR_TXE) != (SPI_SR_TXE));

  /* Transmit Data*/
#if defined (__GNUC__)
  __IO uint8_t *spidr = ((__IO uint8_t *)&SUBGHZSPI->DR);
  *spidr = SUBGHZ_DUMMY_DATA;
#else
  *((__IO uint8_t *)&SUBGHZSPI->DR) = SUBGHZ_DUMMY_DATA;
#endif /* __GNUC__ */

  /* Handle Rx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = ((SystemCoreClock*28U)>>19U)*100;

  /* Wait until RXNE flag is set */
  do {
    if (count == 0U){
      break;
    }
    count--;
  } while ((SUBGHZSPI->SR & SPI_SR_RXNE) != (SPI_SR_RXNE));

  /* Retrieve pData */
  *pData = (uint8_t)(READ_REG(SUBGHZSPI->DR));
}


void rfl_radio_wake_up(void){
	PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select
	uint32_t count = ((SystemCoreClock*24U)>>16U);
	do {
		count--;
	} while(count != 0UL);

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect
}


void rfl_SUBGHZ_WaitOnBusy(void){
  __IO uint32_t count;
  count  = ((SystemCoreClock*28U)>>19U) * ((SystemCoreClock*24U)>>20U);

  /* Wait until Busy signal is set */
  do {
    if(count == 0U){
      break;
    }
    count--;

  } while (((PWR->SR2& PWR_SR2_RFBUSYS) == (PWR_SR2_RFBUSYS)) & ((PWR->SR2 & PWR_SR2_RFBUSYMS) == (PWR_SR2_RFBUSYMS)));

}


void rfl_SUBGHZ_WriteRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size){
  PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit(SUBGHZ_RADIO_WRITE_REGISTER);
	rfl_SUBGHZSPI_Transmit((uint8_t)((Address & 0xFF00U) >> 8U));
	rfl_SUBGHZSPI_Transmit((uint8_t)(Address & 0x00FFU));

	for (uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Transmit(pBuffer[i]);
	}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect
	rfl_SUBGHZ_WaitOnBusy();

}


void rfl_SUBGHZ_ReadRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size){
  uint8_t *pData = pBuffer;
  PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit(SUBGHZ_RADIO_READ_REGISTER);
	rfl_SUBGHZSPI_Transmit((uint8_t)((Address & 0xFF00U) >> 8U));
	rfl_SUBGHZSPI_Transmit((uint8_t)(Address & 0x00FFU));
	rfl_SUBGHZSPI_Transmit(0U);

	for(uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Receive((pData));
		pData++;
	}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect
	rfl_SUBGHZ_WaitOnBusy();

}


void rfl_SUBGHZ_ExecSetCmd(SUBGHZ_RadioSetCmd_t Command, uint8_t *pBuffer, uint16_t Size){
	PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit((uint8_t)Command);

	for (uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Transmit(pBuffer[i]);
	}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect

	if (Command != RADIO_SET_SLEEP){
		rfl_SUBGHZ_WaitOnBusy();
	}
}


void rfl_SUBGHZ_ExecGetCmd(SUBGHZ_RadioGetCmd_t Command, uint8_t *pBuffer, uint16_t Size){
  uint8_t *pData = pBuffer;

  PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit((uint8_t)Command);

	/* Use to flush the Status (First byte) receive from SUBGHZ as not use */
	rfl_SUBGHZSPI_Transmit(0x00U);

	for (uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Receive(pData);
		pData++;
	}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect

	rfl_SUBGHZ_WaitOnBusy();
}


void rfl_SUBGHZ_WriteBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size){

	PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit(SUBGHZ_RADIO_WRITE_BUFFER);
	rfl_SUBGHZSPI_Transmit(Offset);

	for(uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Transmit(pBuffer[i]);
	}
	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect

	rfl_SUBGHZ_WaitOnBusy();

}


void rfl_SUBGHZ_ReadBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size){
  uint8_t *pData = pBuffer;

  PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS; //NSS = 0, select

	rfl_SUBGHZSPI_Transmit(SUBGHZ_RADIO_READ_BUFFER);
	rfl_SUBGHZSPI_Transmit(Offset);
	rfl_SUBGHZSPI_Transmit(0x00U);

	for(uint16_t i = 0U; i < Size; i++){
		rfl_SUBGHZSPI_Receive(pData);
		pData++;
	}

	PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS; //NSS = 1, unselect

	rfl_SUBGHZ_WaitOnBusy();

}
