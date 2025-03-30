/*
 * rfl_subghz.c
 *
 *  Created on: Mar 30, 2025
 *      Author: patry
 */

#include "stm32wlxx.h"

#include "rfl_clocks.h"
#include "rfl_gpio.h"
#include "rfl_subghz_spi.h"


#include <stdio.h>

typedef enum {
	TX,
	RX,
} RFMode;

typedef enum {
	//LSB
    IRQ_TX_DONE = 0,           // 0: Packet transmission finished
    IRQ_RX_DONE = 1,           // 1: Packet reception finished
    IRQ_PREAMBLE_DETECTED = 2, // 2: Preamble detected
    IRQ_SYNC_DETECTED = 3,     // 3: Synchronization word valid
    IRQ_HEADER_VALID = 4,      // 4: Header valid
    IRQ_HEADER_ERR = 5,        // 5: Header CRC error
    IRQ_ERR = 6,               // 6: General error (preamble, syncword, address, CRC, length) / CRC error (LoRa)
    IRQ_CAD_DONE = 7,          // 7: Channel activity detection finished
	//MSB
    IRQ_CAD_DETECTED = 0,      // 8: Channel activity detected
    IRQ_TIMEOUT = 1,           // 9: RX or TX timeout
} IRQSource;


void subghz_clear_irq(void){
	static uint8_t RadioParam[2]  = {0xFF, 0xFF}; //clear all irq status flag

	subghz_spi_getCmd(RADIO_CLR_IRQSTATUS, RadioParam, 2);
}


void rfl_subghz_rfmode(RFMode set_rf_mode){
	switch(set_rf_mode){
	case TX:
		RFR_state(set);
		RFT_state(reset);
		break;
	case RX:
		RFR_state(reset);
		RFT_state(set);
		break;
	}
}


void rfl_subghz_init(void){
	static uint8_t RadioParam[4];

	rfl_radio_spiInit(SUBGHZSPI_BAUDRATEPRESCALER_4);

	//Set tcxo
	RadioParam[0] = (uint8_t)0x7; //byte 0 regtcxotrim (2:0 bits) 3.3V
	RadioParam[1] = (uint8_t)0x00;
	RadioParam[2] = (uint8_t)0x19;
	RadioParam[3] = (uint8_t)0x00; // byte 1-3 timeout 0x001900 - 100 ms
	subghz_spi_setCmd(RADIO_SET_TCXOMODE, RadioParam, 4);

	// 5.9.1. Basic sequence for LoRa TX page 204
	// 5.9.2. Basic sequence for LoRa RX page 205
	// 1. Set Buffer Address
	RadioParam[0] = 0x80U; // Tx base address
	RadioParam[1] = 0x00U; // Rx base address in subghz mode fifo memory
	// NOTE: Need's to find out where is fifo schematic in documentation

	subghz_spi_setCmd(RADIO_SET_BUFFERBASEADDRESS, RadioParam, 2);

}

void rfl_subghz_initTx(void){
	static uint8_t buffer[32] = {0};
	static uint8_t RadioParam[8]  = {0x00};
	static uint8_t interrupts[3] = {0};

	// 3. Set Packet Type
	RadioParam[0] = 0x01U; //LoRa packet type
	subghz_spi_setCmd(RADIO_SET_PACKETTYPE, RadioParam, 1);

	// 4. Set Frame Format
	RadioParam[0] = 0x00U; // PbLength MSB - 12-symbol-long preamble sequence
	RadioParam[1] = 0x0CU; // PbLength LSB - 12-symbol-long preamble sequence
	RadioParam[2] = 0x00U; // explicit header type
	RadioParam[3] = 0x40U; // 64 bit payload length.
	RadioParam[4] = 0x01U; // CRC enabled
	RadioParam[5] = 0x00U; // standard IQ setup
	subghz_spi_setCmd(RADIO_SET_PACKETPARAMS, RadioParam, 6);

	// 5. Define synchronisation word
	RadioParam[0] = 0x14U; // LoRa private network
	RadioParam[1] = 0x24U; // LoRa private network
	subghz_spi_writeRegisters((uint16_t) 0x740, RadioParam, 2);

	// 6. Define RF Frequency
	RadioParam[0] = 0x36U; //RF frequency - 868000000Hz
	RadioParam[1] = 0x40U; //RF frequency - 868000000Hz
	RadioParam[2] = 0x10U; //RF frequency - 868000000Hz
	RadioParam[3] = 0x17U; //RF frequency - 868000000Hz
	subghz_spi_setCmd(RADIO_SET_RFFREQUENCY, RadioParam, 4);

	// 7. Set PA Config
	RadioParam[0] = 0x04U; // PaDutyCycle
	RadioParam[1] = 0x07U; // HpMax
	RadioParam[2] = 0x00U; // HP PA selected
	RadioParam[3] = 0x01U; // predefined in RM0461 and RM0453
	subghz_spi_setCmd(RADIO_SET_PACONFIG, RadioParam, 4);

	// 8.  Set Tx Parameters
	RadioParam[0] = 0x16U; // Power - +22dB
	RadioParam[1] = 0x04U; // RampTime - 200us
	subghz_spi_setCmd(RADIO_SET_TXPARAMS, RadioParam, 2);

	// 9. Set Modulation parameter
	RadioParam[0] = 0x07U; // SF (Spreading factor) - 7 (default)
	RadioParam[1] = 0x09U; // BW (Bandwidth) - 20.83kHz
	RadioParam[2] = 0x01U; // CR (Forward error correction coding rate) - 4/5
	RadioParam[3] = 0x00U; // LDRO (Low data rate optimization) - off
	subghz_spi_setCmd(RADIO_SET_MODULATIONPARAMS, RadioParam, 4);


	// 10. Configure interrupts
	RadioParam[0] = (1 << IRQ_TIMEOUT); // IRQ Mask MSB - Timeout interrupt
	RadioParam[1] = (1 << IRQ_TX_DONE); // IRQ Mask LSB - Tx done interrupt
	RadioParam[2] = 0x00U; // IRQ1 Line Mask MSB
	RadioParam[3] = (1 << IRQ_TX_DONE); // IRQ1 Line Mask LSB - Tx done interrupt on IRQ line 1
	RadioParam[4] = (1 << IRQ_TIMEOUT); // IRQ2 Line Mask MSB - Timeout interrupt on IRQ line 2
	RadioParam[5] = 0x00U; // IRQ2 Line Mask LSB
	RadioParam[6] = 0x00U; // IRQ3 Line Mask MSB
	RadioParam[7] = 0x00U; // IRQ3 Line Mask LSB
	subghz_spi_setCmd(RADIO_CFG_DIOIRQ, RadioParam, 8);

	// 10.1 Read Interrupts
	subghz_spi_getCmd(RADIO_GET_IRQSTATUS, interrupts, 3);

	// Clear interrupts (for sure)
	subghz_clear_irq();

	// 11. Set Tx
	RadioParam[0] = 0x00U;
	RadioParam[1] = 0x00U;
	RadioParam[2] = 0x00U; // Timeout disabled

	//send payload (hello world)
	subghz_spi_setCmd(RADIO_SET_TX, RadioParam, 3);

	//send sin wave
//	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_TXCONTINUOUSWAVE, RadioParam, 0) != HAL_OK)
//	{
//		Subghz_Error_Handler();
//	}
	subghz_spi_getCmd(RADIO_GET_STATUS, buffer, 2);
}

void rfl_subghz_initRx(void){
	static uint8_t RadioParam[8]  = {0x00};

	rfl_subghz_rfmode(RX); //set gpio for rf switches

	// 3. Set Packet Type
	RadioParam[0] = 0x01U; //LoRa packet type
	subghz_spi_setCmd(RADIO_SET_PACKETTYPE, RadioParam, 1);

	// 4. Set Frame Format
	RadioParam[0] = 0x00U; // PbLength MSB - 12-symbol-long preamble sequence
	RadioParam[1] = 0x0CU; // PbLength LSB - 12-symbol-long preamble sequence
	RadioParam[2] = 0x00U; // explicit header type
	RadioParam[3] = 0x40U; // 64 bit payload length.
	RadioParam[4] = 0x01U; // CRC enabled
	RadioParam[5] = 0x00U; // standard IQ setup
	subghz_spi_setCmd(RADIO_SET_PACKETPARAMS, RadioParam, 6);

	// 5. Define synchronisation word
	RadioParam[0] = 0x14U; // LoRa private network
	RadioParam[1] = 0x24U; // LoRa private network
	subghz_spi_writeRegisters((uint16_t) 0x740, RadioParam, 2);

	// 6. Define RF Frequency
	RadioParam[0] = 0x36U; //RF frequency - 868000000Hz
	RadioParam[1] = 0x40U; //RF frequency - 868000000Hz
	RadioParam[2] = 0x10U; //RF frequency - 868000000Hz
	RadioParam[3] = 0x17U; //RF frequency - 868000000Hz
	subghz_spi_setCmd(RADIO_SET_RFFREQUENCY, RadioParam, 4);

	// 9. Set Modulation parameter
	RadioParam[0] = 0x07U; // SF (Spreading factor) - 7 (default)
	RadioParam[1] = 0x09U; // BW (Bandwidth) - 20.83kHz
	RadioParam[2] = 0x01U; // CR (Forward error correction coding rate) - 4/5
	RadioParam[3] = 0x00U; // LDRO (Low data rate optimization) - off
	subghz_spi_setCmd(RADIO_SET_MODULATIONPARAMS, RadioParam, 4);

	// 10. Configure interrupts
	RadioParam[0] = (1 << IRQ_TIMEOUT); // IRQ Mask MSB - Timeout interrupt
	RadioParam[1] = (1 << IRQ_RX_DONE); // IRQ Mask LSB - Tx done interrupt
	RadioParam[2] = 0x00U; // IRQ1 Line Mask MSB
	RadioParam[3] = (1 << IRQ_RX_DONE); // IRQ1 Line Mask LSB - Tx done interrupt on IRQ line 1
	RadioParam[4] = (1 << IRQ_TIMEOUT); // IRQ2 Line Mask MSB - Timeout interrupt on IRQ line 2
	RadioParam[5] = 0x00U; // IRQ2 Line Mask LSB
	RadioParam[6] = 0x00U; // IRQ3 Line Mask MSB
	RadioParam[7] = 0x00U; // IRQ3 Line Mask LSB
	subghz_spi_setCmd(RADIO_CFG_DIOIRQ, RadioParam, 8);

	// Clear interrupts (for sure)
	subghz_clear_irq();

	// 7. Set PA Config
	RadioParam[0] = 0x04U; // PaDutyCycle
	RadioParam[1] = 0x07U; // HpMax
	RadioParam[2] = 0x00U; // HP PA selected
	RadioParam[3] = 0x01U; // predefined in RM0461 and RM0453
	subghz_spi_setCmd(RADIO_SET_PACONFIG, RadioParam, 4);

	// 8.  Set Tx Parameters
	RadioParam[0] = 0x16U; // Power - +22dB
	RadioParam[1] = 0x04U; // RampTime - 200us
	subghz_spi_setCmd(RADIO_SET_TXPARAMS, RadioParam, 2);
}

void subghz_receive(){
	uint8_t read_payload[64] = {0};
	subghz_spi_readBuffer(0x00U, read_payload, sizeof(read_payload));
	printf("Received: %s\r\n", read_payload);
}


void rfl_subghz_transmit(uint8_t payload[64]){
	uint8_t RadioParam[8]  = {0x00};
	rfl_subghz_rfmode(TX); //set gpio for rf switches
	RadioParam[0] = 0x00U;
	RadioParam[1] = 0x00U;
	RadioParam[2] = 0x00U; // Timeout disabled
	subghz_spi_setCmd(RADIO_SET_STANDBY, RadioParam, 3);
	//Write Payload to Buffer
	subghz_spi_writeBuffer(0x80U, payload, 64);
	subghz_spi_setCmd(RADIO_SET_TX, RadioParam, 3);
}

void rfl_subghz_setRxContinous(void){
	uint8_t RadioParam[8]  = {0x00};
	rfl_subghz_rfmode(RX); //set gpio for rf switches
	RadioParam[0] = 0x00U;
	RadioParam[1] = 0x00U;
	RadioParam[2] = 0x00U; // Timeout disabled
	subghz_spi_setCmd(RADIO_SET_STANDBY, RadioParam, 3);
	subghz_spi_setCmd(RADIO_SET_RX, RadioParam, 3);
}


void subghz_handle_irq(void){
//	HAL_SUBGHZ_IRQHandler(&subghz_handler);
	subghz_receive();
}
