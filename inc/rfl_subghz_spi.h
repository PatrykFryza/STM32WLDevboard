/*
 * rfl_subghz.h
 *
 *  Created on: Mar 28, 2025
 *      Author: patry
 */

#ifndef INC_RFL_SUBGHZ_H_
#define INC_RFL_SUBGHZ_H_

#include "rfl_types.h"


#define SUBGHZSPI_BAUDRATEPRESCALER_2       (0x00000000U)
#define SUBGHZSPI_BAUDRATEPRESCALER_4       (SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_8       (SPI_CR1_BR_1)
#define SUBGHZSPI_BAUDRATEPRESCALER_16      (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_32      (SPI_CR1_BR_2)
#define SUBGHZSPI_BAUDRATEPRESCALER_64      (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SUBGHZSPI_BAUDRATEPRESCALER_128     (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SUBGHZSPI_BAUDRATEPRESCALER_256     (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

void rfl_radio_reset(void);
void rfl_radio_wakeUp(void);
void rfl_radio_spiInit(uint32_t baudratePrescaler);
void subghz_spi_writeRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size);
void subghz_spi_readRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size);
void subghz_spi_setCmd(SUBGHZ_RadioSetCmd_t Command, uint8_t *pBuffer, uint16_t Size);
void subghz_spi_getCmd(SUBGHZ_RadioGetCmd_t Command, uint8_t *pBuffer, uint16_t Size);
void subghz_spi_writeBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size);
void subghz_spi_readBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size);

#endif /* INC_RFL_SUBGHZ_H_ */
