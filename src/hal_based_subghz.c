/*
 * hal_based.subghz.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Damian Fryza
 */

#include "hal_based_subghz.h"

#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_gpio.h"
#include "stm32wlxx_hal_subghz.h"

#include "rfl_clocks.h"
#include "rfl_gpio.h"

static void Subghz_Error_Handler(void){
	while(1){
		__NOP();
	}
}

SUBGHZ_HandleTypeDef subghz_handler;

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

void subghz_rfmode(RFMode set_rf_mode){
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

void subghz_init(void){
	//turn on tcxo for radio
	HSE32_Radio_init();

	//initialize gpio for rfswitch
	rfswitch_init();

	//radio reset
	RCC->CSR |= RCC_CSR_RFRST;
	HAL_Delay(5);
	RCC->CSR &= ~RCC_CSR_RFRST;
	while(RCC->CSR & RCC_CSR_RFRSTF); //wait for reset flag clear

	//subghz spi init
	subghz_handler.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
	__HAL_RCC_SUBGHZSPI_CLK_ENABLE();

	/* SUBGHZ interrupt Init */
	HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
	if(HAL_SUBGHZ_Init(&subghz_handler) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	static uint8_t RadioParam[4];

	//Set tcxo
	RadioParam[0] = (uint8_t)0x7; //byte 0 regtcxotrim (2:0 bits) 3.3V
	RadioParam[1] = (uint8_t)0x00;
	RadioParam[2] = (uint8_t)0x19;
	RadioParam[3] = (uint8_t)0x00; // byte 1-3 timeout 0x001900 - 100 ms
	HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_TCXOMODE, RadioParam, 4);

	// 5.9.1. Basic sequence for LoRa TX page 204
	// 5.9.2. Basic sequence for LoRa RX page 205
	// 1. Set Buffer Address
	RadioParam[0] = 0x80U; // Tx base address
	RadioParam[1] = 0x00U; // Rx base address in subghz mode fifo memory
	// NOTE: Need's to find out where is fifo schematic in documentation

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_BUFFERBASEADDRESS, RadioParam, 2) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

}

void subghz_init_tx(void){
	static uint8_t buffer[32] = {0};
	static uint8_t payload[64] = "Hello world!\r\n";
	static uint8_t RadioParam[8]  = {0x00};
	static uint8_t interrupts[3] = {0};

	subghz_rfmode(TX); //set gpio for rf switches

	// 2. Write Payload to Buffer
	if (HAL_SUBGHZ_WriteBuffer(&subghz_handler, 0x80U, payload, sizeof(payload)) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	// 3. Set Packet Type
	RadioParam[0] = 0x01U; //LoRa packet type
	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_PACKETTYPE, RadioParam, 1) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 4. Set Frame Format
	RadioParam[0] = 0x00U; // PbLength MSB - 12-symbol-long preamble sequence
	RadioParam[1] = 0x0CU; // PbLength LSB - 12-symbol-long preamble sequence
	RadioParam[2] = 0x00U; // explicit header type
	RadioParam[3] = 0x40U; // 64 bit payload length.
	RadioParam[4] = 0x01U; // CRC enabled
	RadioParam[5] = 0x00U; // standard IQ setup

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_PACKETPARAMS, RadioParam, 6) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 5. Define synchronisation word
	RadioParam[0] = 0x14U; // LoRa private network
	RadioParam[1] = 0x24U; // LoRa private network


	if (HAL_SUBGHZ_WriteRegisters(&subghz_handler, (uint16_t) 0x740, RadioParam, 2) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 6. Define RF Frequency
	RadioParam[0] = 0x36U; //RF frequency - 868000000Hz
	RadioParam[1] = 0x40U; //RF frequency - 868000000Hz
	RadioParam[2] = 0x10U; //RF frequency - 868000000Hz
	RadioParam[3] = 0x17U; //RF frequency - 868000000Hz


	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_RFFREQUENCY, RadioParam, 4) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 7. Set PA Config
	RadioParam[0] = 0x04U; // PaDutyCycle
	RadioParam[1] = 0x07U; // HpMax
	RadioParam[2] = 0x00U; // HP PA selected
	RadioParam[3] = 0x01U; // predefined in RM0461 and RM0453

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_PACONFIG, RadioParam, 4) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 8.  Set Tx Parameters
	RadioParam[0] = 0x16U; // Power - +22dB
	RadioParam[1] = 0x04U; // RampTime - 200us

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_TXPARAMS, RadioParam, 2) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 9. Set Modulation parameter
	RadioParam[0] = 0x07U; // SF (Spreading factor) - 7 (default)
	RadioParam[1] = 0x09U; // BW (Bandwidth) - 20.83kHz
	RadioParam[2] = 0x01U; // CR (Forward error correction coding rate) - 4/5
	RadioParam[3] = 0x00U; // LDRO (Low data rate optimization) - off

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_MODULATIONPARAMS, RadioParam, 4) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 10. Configure interrupts
	RadioParam[0] = (1 << IRQ_TIMEOUT); // IRQ Mask MSB - Timeout interrupt
	RadioParam[1] = (1 << IRQ_TX_DONE); // IRQ Mask LSB - Tx done interrupt
	RadioParam[2] = 0x00U; // IRQ1 Line Mask MSB
	RadioParam[3] = (1 << IRQ_TX_DONE); // IRQ1 Line Mask LSB - Tx done interrupt on IRQ line 1
	RadioParam[4] = (1 << IRQ_TIMEOUT); // IRQ2 Line Mask MSB - Timeout interrupt on IRQ line 2
	RadioParam[5] = 0x00U; // IRQ2 Line Mask LSB
	RadioParam[6] = 0x00U; // IRQ3 Line Mask MSB
	RadioParam[7] = 0x00U; // IRQ3 Line Mask LSB

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_CFG_DIOIRQ, RadioParam, 8) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 10.1 Read Interrupts
	if (HAL_SUBGHZ_ExecGetCmd(&subghz_handler, RADIO_GET_IRQSTATUS, interrupts, 3) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	// Clear interrupts (for sure)
	subghz_clear_irq();

	// 11. Set Tx
	RadioParam[0] = 0x00U;
	RadioParam[1] = 0x00U;
	RadioParam[2] = 0x00U; // Timeout disabled

	//send payload (hello world)
	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_TX, RadioParam, 3) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	//send sin wave
//	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_TXCONTINUOUSWAVE, RadioParam, 0) != HAL_OK)
//	{
//		Subghz_Error_Handler();
//	}

	HAL_Delay(500);
	HAL_SUBGHZ_ExecGetCmd(&subghz_handler, RADIO_GET_STATUS, buffer, 2);
}

void subghz_init_rx(void){
	static uint8_t RadioParam[8]  = {0x00};

	subghz_rfmode(RX); //set gpio for rf switches

	// 2. Set Packet Type
	RadioParam[0] = 0x01U; //LoRa packet type
	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_PACKETTYPE, RadioParam, 1) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 3. Set Frame Format
	RadioParam[0] = 0x00U; // PbLength MSB - 12-symbol-long preamble sequence
	RadioParam[1] = 0x0CU; // PbLength LSB - 12-symbol-long preamble sequence
	RadioParam[2] = 0x00U; // explicit header type
	RadioParam[3] = 0x40U; // 64 bit payload length.
	RadioParam[4] = 0x01U; // CRC enabled
	RadioParam[5] = 0x00U; // standard IQ setup

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_PACKETPARAMS, RadioParam, 6) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 4. Define synchronisation word
	RadioParam[0] = 0x14U; // LoRa private network
	RadioParam[1] = 0x24U; // LoRa private network


	if (HAL_SUBGHZ_WriteRegisters(&subghz_handler, (uint16_t) 0x740, RadioParam, 2) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 5. Define RF Frequency
	RadioParam[0] = 0x36U; //RF frequency - 868000000Hz
	RadioParam[1] = 0x40U; //RF frequency - 868000000Hz
	RadioParam[2] = 0x10U; //RF frequency - 868000000Hz
	RadioParam[3] = 0x17U; //RF frequency - 868000000Hz


	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_RFFREQUENCY, RadioParam, 4) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	// 6. Set Modulation parameter
	RadioParam[0] = 0x07U; // SF (Spreading factor) - 7 (default)
	RadioParam[1] = 0x09U; // BW (Bandwidth) - 20.83kHz
	RadioParam[2] = 0x01U; // CR (Forward error correction coding rate) - 4/5
	RadioParam[3] = 0x00U; // LDRO (Low data rate optimization) - off

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_MODULATIONPARAMS, RadioParam, 4) != HAL_OK)
	{
		Subghz_Error_Handler();
	}


	// 7. Configure interrupts
	RadioParam[0] = (1 << IRQ_TIMEOUT); // IRQ Mask MSB - Timeout interrupt
	RadioParam[1] = (1 << IRQ_RX_DONE); // IRQ Mask LSB - Rx done interrupt
	RadioParam[2] = 0x00U; // IRQ1 Line Mask MSB
	RadioParam[3] = (1 << IRQ_RX_DONE); // Rx done on IRQ1
	RadioParam[4] = (1 << IRQ_TIMEOUT); // Timeout on IRQ2
	RadioParam[5] = 0x00U; // IRQ2 Line Mask LSB
	RadioParam[6] = 0x00U; // IRQ3 Line Mask MSB
	RadioParam[7] = 0x00U; // IRQ3 Line Mask LSB

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_CFG_DIOIRQ, RadioParam, 8) != HAL_OK)
	{
		Subghz_Error_Handler();
	}

	// Clear interrupts (for sure)
	subghz_clear_irq();

	// 8. Set rx mode
	RadioParam[0] = 0x00U;
	RadioParam[1] = 0x00U;
	RadioParam[2] = 0x00U; // no timeout (continues mode)

	if (HAL_SUBGHZ_ExecSetCmd(&subghz_handler, RADIO_SET_RX, RadioParam, 3) != HAL_OK)
	{
		Subghz_Error_Handler();
	}
}

void subghz_clear_irq(void){
	static uint8_t RadioParam[2]  = {0xFF, 0xFF}; //clear all irq status flag

	if (HAL_SUBGHZ_ExecGetCmd(&subghz_handler, RADIO_CLR_IRQSTATUS, RadioParam, 2) != HAL_OK)
	{
		Subghz_Error_Handler();
	}
}

void subghz_handle_irq(void){
	HAL_SUBGHZ_IRQHandler(&subghz_handler);
}

























