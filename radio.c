#include <stdio.h>
#include <string.h>

#include "rf.h"
#include "main.h"

#define SENDTMO		301 // 300*10us + 1 = 3ms

#if EN_AES
#define AES_TINY	1
#if AES_TINY
#include "tiny-AES128/include/aes.h"
#else
#include "aes/include/aes.h"
#include "aes/include/aes_user_options.h"
aes_data_t aes_data;
#endif
MESSAGE_T aesbuf;
#endif

///////////////////////////////////////////
// Inline function definitions
///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// static __INLINE uint8_t enc_dec_accel_galois_multiply(uint8_t a, uint8_t b) __reentrant
//
// Description:
//  Performs a GF(2^8) mutliplication in hardware and returns the result
//
// Parameters:
//  uint8_t a - multiplier in the operation
//  uint8_t b - multiplicand in the operation
//
// Return value:
//  Result of the GF(2^8) multiplication
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//See enc_dec_accel_galois_multiply.c for function details (function body is the same)
unsigned char enc_dec_accel_galois_multiply(unsigned char a, unsigned char b)
{
	//Load the hardware registers for the operators
	CCPDATIA = a;
	CCPDATIB = b;

	//Return the result of the hardware GF(2^8) multiplication
	return CCPDATO;
}

void radio_init() {
	uint8_t setup;

#if EN_AES && ! AES_TINY
	if (config.useaes) {
		aes_initialize(&aes_data, AES_KEY_LENGTH_128_BITS, config.aeskey, NULL);
	}
#endif

	rf_spi_configure_enable();

	setup = RF_CONFIG_EN_CRC | RF_CONFIG_CRCO | RF_CONFIG_PWR_UP;		// power up here, CRC 2 bytes
	rf_write_register(RF_CONFIG, &setup, 1) ;

	delay_ms(2);

	setup = RF_SETUP_RETR_ARD_1000 | RF_SETUP_RETR_ARC_3; 		// 1000us, 3 retry
	rf_write_register(RF_SETUP_RETR, &setup, 1);

	setup = RF_SETUP_AW_5BYTES;									// 5 bytes address
	rf_write_register(RF_SETUP_AW, &setup, 1);

	setup = RF_EN_RXADDR_ERX_P0 | RF_EN_RXADDR_ERX_P1;
	rf_write_register(RF_EN_RXADDR, &setup, 1);

	rf_write_register(RF_RX_ADDR_P0, config.srvaddr, 5);
	rf_write_register(RF_TX_ADDR, config.srvaddr, 5);

	config.devaddr[4] = config.deviceID;
	rf_write_register(RF_RX_ADDR_P1, config.devaddr, 5);

	setup = MSGLEN;												/* Number of bytes in RX payload		*/
	rf_write_register(RF_RX_PW_P0, &setup, 1);
	rf_write_register(RF_RX_PW_P1, &setup, 1);

	setup = 0;
	if (config.autoask)
		setup = RF_EN_AA_ENAA_P0 | RF_EN_AA_ENAA_P1;			// set autoask
	rf_write_register(RF_EN_AA, &setup, 1);

	rf_write_register(RF_RF_CH, &config.channel, 1);			// set channel

	setup = RF_RF_SETUP_RF_PWR_0_DBM;							// 0 dbm power, 1Mhz
	setup &= ~(RF_RF_SETUP_RF_DR_LOW | RF_RF_SETUP_RF_DR_HIGH);	// default datarate = 2, 1Mhz

	if(config.datarate == 1)
		setup |= RF_RF_SETUP_RF_DR_LOW;
	else if (config.datarate == 3)
		setup |= RF_RF_SETUP_RF_DR_HIGH;

	rf_write_register(RF_RF_SETUP, &setup, 1);

	rf_set_as_rx(true);											 //change the device to an RX to get the character back from the other 24L01
}

uint8_t rfsend(const MESSAGE_T *msg) {
	uint16_t timeout;
	uint8_t retry = config.maxsend;
#if EN_AES
	if (config.useaes) {
#if AES_TINY
		AES128_ECB_encrypt((uint8_t*) msg, config.aeskey, (uint8_t*) aesbuf);
#else
		aes_encrypt_ecb(&aes_data, (unsigned char *) msg, (unsigned char *) aesbuf);
#endif
	}
	else {
		memcpy(&aesbuf, msg, MSGLEN);
	}
#endif
	rf_set_as_tx(); //resume normal operation as a TX

start:
	timeout = SENDTMO;
	if (rf_tx_fifo_is_full()) rf_flush_tx();
	rf_irq_clear_all();

#if EN_AES
	rf_write_tx_payload((uint8_t*) aesbuf, MSGLEN, true); //transmit received char over RF
#else
	rf_write_tx_payload((uint8_t*) msg, MSGLEN, true); //transmit received char over RF
#endif

	//wait until the packet has been sent or the maximum number of retries has been reached
	while(--timeout) {
		if (rf_irq_pin_active() && (rf_irq_tx_ds_active() || rf_irq_max_rt_active())) break;
		delay_us(10);	// 10us
	}

	if (rf_irq_pin_active() && rf_irq_tx_ds_active()) {   // checking tx_ds bit
#if DEBUG
		printf("received tx_ds\r\n");
#endif
		rf_irq_clear_all();
		rf_set_as_rx(true);								  //change the device to an RX to get the character back from the other 24L01
		return true;
	}

	if (rf_irq_pin_active() && rf_irq_max_rt_active()) {  // checking max_rt bit
#if DEBUG
		printf("received max_rt\r\n");
#endif
		if (--retry) goto start;
	}

	if (!timeout) { // checking timeout if nothing
#if DEBUG
		printf("no irq received\r\n");
#endif
		if (--retry) goto start;
	}

	rf_irq_clear_all();
	rf_set_as_rx(true);	//change the device to an RX to get the character back from the other 24L01
	return false;
}

// blocking read NRF24LE1, timeout in 10us intervals
uint8_t rfread(MESSAGE_T *msg, uint16_t timeout) {
	uint8_t status, state = 0;

	rf_irq_clear_all();
	while (--timeout) {
		if(rf_irq_pin_active() && rf_irq_rx_dr_active()) {
#if EN_AES
			status = rf_read_rx_payload((uint8_t *) aesbuf, MSGLEN); //get the payload into data
#else
			status = rf_read_rx_payload((uint8_t *) msg, MSGLEN); 	 //get the payload into data
#endif
			if (rf_is_rxed_payload_on_pipe_1_in_status_val(status)) {
				state = 1;
				break;
			}
		}
		delay_us(10);	// 10us
	}
#if EN_AES
	if (state) {
		if (config.useaes) {
#if AES_TINY
			AES128_ECB_decrypt((uint8_t*) aesbuf, config.aeskey, (uint8_t*) msg);
#else
			aes_decrypt_ecb(&aes_data, (unsigned char *) aesbuf, (unsigned char *) msg);
#endif
		}
		else {
			memcpy(msg, &aesbuf, MSGLEN);
		}
	}
#endif
	rf_irq_clear_all(); //clear interrupts again
	return state;
}

// non blocking read NRF24LE1 queue
uint8_t rfreadqueue(MESSAGE_T *msg) {
	uint8_t status;

	rf_irq_clear_all(); //clear interrupts again
#if EN_AES
	status = rf_read_rx_payload((uint8_t *) aesbuf, MSGLEN); //get the payload into data
#else
	status = rf_read_rx_payload((uint8_t *) msg, MSGLEN); 	 //get the payload into data
#endif
	if (! rf_is_rxed_payload_on_pipe_1_in_status_val(status)) {
		return false;
	}

#if EN_AES
	if (config.useaes) {
#if AES_TINY
		AES128_ECB_decrypt((uint8_t*) aesbuf, config.aeskey, (uint8_t*) msg);
#else
		aes_decrypt_ecb(&aes_data, (unsigned char *) aesbuf, (unsigned char *) msg);
#endif
	} else {
		memcpy(msg, &aesbuf, MSGLEN);
	}
#endif

	return true;
}

void rfpwrDown(void) {
	rf_irq_clear_all();
    rf_power_down();
}
