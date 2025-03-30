/*
 * rfl_subghz.h
 *
 *  Created on: Mar 28, 2025
 *      Author: patry
 */

#ifndef INC_RFL_SUBGHZ_H_
#define INC_RFL_SUBGHZ_H_

typedef enum
{
  RADIO_SET_SLEEP                           = 0x84U,
  RADIO_SET_STANDBY                         = 0x80U,
  RADIO_SET_FS                              = 0xC1U,
  RADIO_SET_TX                              = 0x83U,
  RADIO_SET_RX                              = 0x82U,
  RADIO_SET_RXDUTYCYCLE                     = 0x94U,
  RADIO_SET_CAD                             = 0xC5U,
  RADIO_SET_TXCONTINUOUSWAVE                = 0xD1U,
  RADIO_SET_TXCONTINUOUSPREAMBLE            = 0xD2U,
  RADIO_SET_PACKETTYPE                      = 0x8AU,
  RADIO_SET_RFFREQUENCY                     = 0x86U,
  RADIO_SET_TXPARAMS                        = 0x8EU,
  RADIO_SET_PACONFIG                        = 0x95U,
  RADIO_SET_CADPARAMS                       = 0x88U,
  RADIO_SET_BUFFERBASEADDRESS               = 0x8FU,
  RADIO_SET_MODULATIONPARAMS                = 0x8BU,
  RADIO_SET_PACKETPARAMS                    = 0x8CU,
  RADIO_RESET_STATS                         = 0x00U,
  RADIO_CFG_DIOIRQ                          = 0x08U,
  RADIO_CLR_IRQSTATUS                       = 0x02U,
  RADIO_CALIBRATE                           = 0x89U,
  RADIO_CALIBRATEIMAGE                      = 0x98U,
  RADIO_SET_REGULATORMODE                   = 0x96U,
  RADIO_SET_TCXOMODE                        = 0x97U,
  RADIO_SET_TXFALLBACKMODE                  = 0x93U,
  RADIO_SET_RFSWITCHMODE                    = 0x9DU,
  RADIO_SET_STOPRXTIMERONPREAMBLE           = 0x9FU,
  RADIO_SET_LORASYMBTIMEOUT                 = 0xA0U,
  RADIO_CLR_ERROR                           = 0x07U
} SUBGHZ_RadioSetCmd_t;


typedef enum
{
  RADIO_GET_STATUS                          = 0xC0U,
  RADIO_GET_PACKETTYPE                      = 0x11U,
  RADIO_GET_RXBUFFERSTATUS                  = 0x13U,
  RADIO_GET_PACKETSTATUS                    = 0x14U,
  RADIO_GET_RSSIINST                        = 0x15U,
  RADIO_GET_STATS                           = 0x10U,
  RADIO_GET_IRQSTATUS                       = 0x12U,
  RADIO_GET_ERROR                           = 0x17U
} SUBGHZ_RadioGetCmd_t;


void rfl_radio_reset(void);
void rfl_radio_spi_init(void);
void rfl_SUBGHZ_ExecSetCmd(SUBGHZ_RadioSetCmd_t Command, uint8_t *pBuffer, uint16_t Size);
void rfl_SUBGHZ_ExecGetCmd(SUBGHZ_RadioGetCmd_t Command, uint8_t *pBuffer, uint16_t Size);
void rfl_SUBGHZ_WriteRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size);
void rfl_SUBGHZ_ReadRegisters(uint16_t Address, uint8_t *pBuffer, uint16_t Size);
void rfl_SUBGHZ_ReadBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size);
void rfl_SUBGHZ_WriteBuffer(uint8_t Offset, uint8_t *pBuffer, uint16_t Size);

#endif /* INC_RFL_SUBGHZ_H_ */
