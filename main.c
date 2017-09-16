/*
 *  Created on: 15.04.2017
 *      Author: andru
 *
 *      nRF24LE1 remote switch unit
 *      support AES encryption
 *
 *		based on great nRF24LE1 SDK https://github.com/DeanCording/nRF24LE1_SDK
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gpio.h"
#include "delay.h"
#include "memory.h"
#include "interrupt.h"

#include "main.h"
#include "radio.h"
#include "crc8.h"

#define FIRMWARE		11		// FW VER 0.10
#define MAGIC			0xAE	// magic word

#if DEBUG
#define EN_UART			1	// use UART for debugging
#define UARTTXPIN		GPIO_PIN_ID_P0_3		// P0.3 - UART TX
#define UARTRXPIN		GPIO_PIN_ID_P0_4		// P0.4	- UART RX
#else
#define EN_UART			0	// use UART for debugging
#endif

#if EN_LED
#define LEDPIN			GPIO_PIN_ID_P1_4		// P1.4 - LED
#endif

#define OUTPIN			GPIO_PIN_ID_P1_2		// P1.2 - power control output
#define switch_on		( gpio_pin_val_set(OUTPIN) )
#define switch_off		( gpio_pin_val_clear(OUTPIN) )
#define switch_read		( gpio_pin_val_read(OUTPIN) )

#if EN_SW
#define SWPIN			GPIO_PIN_ID_P1_3		// P1.3 - switch button input
#endif

#define NVM_START_ADDRESS	MEMORY_FLASH_NV_STD_END_START_ADDRESS
#define ENVM_START_ADDRESS	(0xFB00)

#if EN_UART
#include "uart.h"
#endif

#if EN_WDG
#include "watchdog.h"
#define WDGTIMEOUT	1	// watchdog timeout, S
#endif

CONFIG_T config;

// halt
static void halt(void) {
	while (1) {
#if EN_LED
		gpio_pin_val_complement(LEDPIN);
		delay_ms(250);
#endif
	}
}

#if EN_RTC
#include "rtc2.h"
#include "pwr_clk_mgmt.h"
static volatile uint16_t rtc_counter = 1;

static interrupt_isr_rtc2() {
#if EN_LED
	gpio_pin_val_complement(LEDPIN);
#endif
	if (rtc_counter > 0) rtc_counter--;
}
#endif

// read config from eNVM with start address
static uint8_t read_config(uint16_t addr) {
	uint8_t i, pcon_temp;
	uint8_t buf[sizeof(CONFIG_T) + 1];
	CONFIG_T *p;

	interrupt_save_global_flag(PSW_SB_F0);
	interrupt_control_global_disable();

	pcon_temp = PCON;
	memory_movx_accesses_data_memory();

	for (i = 0; i < sizeof(CONFIG_T) + 1; i++) {
		buf[i] = *((__xdata uint8_t *) (addr + i));
	}

	PCON = pcon_temp;
	interrupt_restore_global_flag(PSW_SB_F0);

	p = (CONFIG_T *) &buf;
	if ( p->magic == MAGIC && CRC8(buf, sizeof(CONFIG_T)) == buf[sizeof(CONFIG_T)] ) {
		memcpy((uint8_t *) &config, buf, sizeof(CONFIG_T));
		return 1;
	}
	return 0;
}

// write config to eNVM with start address
static uint8_t write_config(uint16_t addr) {
	uint8_t i, ret, temp;
	uint8_t buf[sizeof(CONFIG_T) + 1];

	memcpy(buf, (uint8_t *) &config, sizeof(CONFIG_T));
	buf[sizeof(CONFIG_T)] = CRC8(buf, sizeof(CONFIG_T));

	if (memory_flash_get_page_num_from_address(addr, &temp) == MEMORY_FLASH_OK) {
		if (memory_flash_erase_page(temp) != MEMORY_FLASH_OK) return 0;
	} else return 0;

	ret = 1;
	interrupt_save_global_flag(PSW_SB_F0);
	interrupt_control_global_disable();

	temp = PCON;
	memory_movx_accesses_data_memory();

	for (i = 0; i < sizeof(CONFIG_T) + 1; i++) {
		memory_flash_enable_write_access();
		*((__xdata uint8_t *) (addr + i)) = buf[i];
		memory_flash_wait_for_write_complete();
		memory_flash_disable_write_access();
		if ( buf[i] != *((__xdata uint8_t *) (addr + i)) ) {
			ret = 0;
			break;
		}
	}

	PCON = temp;
	interrupt_restore_global_flag(PSW_SB_F0);

	return ret;
}

static void send_config(uint8_t addr, uint16_t value) {
	MESSAGE_T message;

#if DEBUG
	printf("addr=%d, value=%d\r\n", addr, (uint16_t) value);
#endif
	message.msgType = SENSOR_INFO;
	message.deviceID = config.deviceID;
	message.firmware = FIRMWARE;
	message.address = addr;
	message.data.iValue = (uint16_t) value;
	rfsend(&message);
}

static void send_config_err(uint8_t addr, uint8_t errcode) {
	MESSAGE_T message;

#if DEBUG
	printf("addr=%d, config error=%d\r\n", addr, errcode);
#endif
	message.msgType = SENSOR_ERROR;
	message.deviceID = config.deviceID;
	message.firmware = FIRMWARE;
	message.address = addr;
	message.error = errcode;
	rfsend(&message);
}

void send_switch(uint8_t state, uint8_t error) {
	MESSAGE_T message;

#if DEBUG
	printf("switch state=%d, error=%d\r\n", state, error);
#endif
	message.deviceID = config.deviceID;
	message.firmware = FIRMWARE;
	message.address = ADDR_SWITCH;
	message.msgType = SENSOR_INFO;
	message.data.iValue = state;
	rfsend(&message);

	if (error) {
		message.msgType = SENSOR_ERROR;
		message.error = error;
		rfsend(&message);
	}
}

void send_button(uint8_t state) {
	MESSAGE_T message;

#if DEBUG
	printf("button state=%d\r\n", state);
#endif
	message.deviceID = config.deviceID;
	message.firmware = FIRMWARE;
	message.address = ADDR_BUTTON;
	message.msgType = SENSOR_INFO;
	message.data.iValue = state;
	rfsend(&message);
}


// main
void main(void) {

	// variable definition
	uint8_t ret, cmd;
    MESSAGE_T message;

#if EN_SW
	uint8_t ex_sw;
#endif

	// program code
#if EN_LED
	gpio_pin_configure(LEDPIN,
		GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT
		| GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET
		| GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
		);
#endif

	// OUTPIN pin configure
	gpio_pin_configure(OUTPIN,
		GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT
		| GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR
		| GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
		);

#if EN_UART
	// Setup UART pins
	gpio_pin_configure(GPIO_PIN_ID_FUNC_RXD,
		GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
		GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
		);

	gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
		GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
		GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
		GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
		);

	uart_configure_8_n_1_38400();
#endif


	if (! read_config(NVM_START_ADDRESS)) {
		// NVM config wrong stop work
		halt();
	}

	ret = config.version;
	if (! read_config(ENVM_START_ADDRESS) || config.version != ret) {
		if (! read_config(NVM_START_ADDRESS)) {
			// NVM config wrong stop work
			halt();
		}
		if (! write_config(ENVM_START_ADDRESS)) {
			// eNVM config write error stop work
			halt();
		}
	}

#if EN_RF
 	radio_init();
#endif

#if EN_SW
	// SWPIN pin configure
	gpio_pin_configure(SWPIN,
			GPIO_PIN_CONFIG_OPTION_DIR_INPUT
			| GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS
	);
	ex_sw = gpio_pin_val_read(SWPIN);
	if (config.enable && ex_sw) {
		switch_on;
	}
	send_button(ex_sw);
	send_switch(switch_read, SWITCH_OK);
#endif

#if EN_RTC
	//CLKLF is not running, so enable RCOSC32K and wait until it is ready
	pwr_clk_mgmt_clklf_configure(PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);
	pwr_clk_mgmt_wait_until_clklf_is_ready();

	rtc2_configure(
		RTC2_CONFIG_OPTION_ENABLE
		| RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ,
		32767
	);			// 1s
	interrupt_control_rtc2_enable();
	interrupt_control_global_enable();
	rtc2_run();
#endif

#if EN_WDG
	watchdog_setup();
	watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));
#endif


 	while(1) {

#if EN_RTC
		if (config.report > 0 && rtc_counter == 0) {
			if (config.enable) {
				send_switch(switch_read, SWITCH_OK);
			} else {
				send_switch(switch_read, SWITCH_DISABLE);
			}
			send_button(ex_sw);
			rtc_counter = config.report;
		}
#endif

#if EN_SW
		cmd = gpio_pin_val_read(SWPIN);
		if (cmd != ex_sw) {
			if (config.enable) {
				if (cmd) {
					switch_on;
				} else {
					switch_off;
				}
				send_switch(switch_read, SWITCH_OK);
			} else {
				send_switch(switch_read, SWITCH_DISABLE);
			}
			ex_sw = cmd;
			send_button(ex_sw);
		}
#endif

WAITCMD:
		// check received command from smarthome gateway
		cmd = rfreadqueue(&message);

		if (cmd && message.deviceID == config.deviceID && message.msgType == SENSOR_CMD) {
#if DEBUG
			printf("\r\ncommand: %d\r\n", message.command);
			printf("address: %d\r\n", message.address);
			printf("param: %d\r\n\r\n", (uint16_t) message.data.iValue);
#endif
			// команда управления выключателем
			if (message.address == ADDR_SWITCH) {
				switch (message.command) {
				case CMD_ON:
					if (config.enable) {
						switch_on;
						send_switch(switch_read, SWITCH_OK);
					} else {
						send_switch(switch_read, SWITCH_DISABLE);
					}
					break;
				case CMD_OFF:
					if (config.enable) {
						switch_off;
						send_switch(switch_read, SWITCH_OK);
					} else {
						send_switch(switch_read, SWITCH_DISABLE);
					}
					break;
				default:
					break;
				}
			// команда чтения/записи конфигурации
			} else if ((message.command == CMD_CFGREAD || message.command == CMD_CFGWRITE)) {
				switch (message.address) {
				case CFG_MAXSEND:
					if (message.command == CMD_CFGREAD) {
						send_config(CFG_MAXSEND, config.maxsend);
					} else {
						if (message.data.iValue == 0 || message.data.iValue > 50) {
						    send_config_err(CFG_MAXSEND, CFGSET_PARAM);
						    break;
						}
						config.maxsend = (uint8_t) message.data.iValue;
						if (!write_config(ENVM_START_ADDRESS))	{
							send_config_err(CFG_MAXSEND, CFGSET_WRITE);
							break;
						}
						send_config(CFG_MAXSEND, config.maxsend);
					}
					break;
				case CFG_ENABLE:
					if (message.command == CMD_CFGREAD) {
						send_config(CFG_ENABLE, config.enable);
					} else {
						if (message.data.iValue > 1) {
						    send_config_err(CFG_ENABLE, CFGSET_PARAM);
						    break;
						}
						config.enable = (uint8_t) message.data.iValue;
						if (!write_config(ENVM_START_ADDRESS))	{
							send_config_err(CFG_ENABLE, CFGSET_WRITE);
							break;
						}
						send_config(CFG_ENABLE, config.enable);
					}
					break;
				case CFG_REPORT:
					if (message.command == CMD_CFGREAD) {
						send_config(CFG_REPORT, config.report);
					} else {
						if (message.data.iValue > 65000) {
						    send_config_err(CFG_REPORT, CFGSET_PARAM);
						    break;
						}
						config.report = (uint16_t) message.data.iValue;
						if (!write_config(ENVM_START_ADDRESS))	{
							send_config_err(CFG_REPORT, CFGSET_WRITE);
							break;
						}
						send_config(CFG_REPORT, config.report);
					}
					break;
				default:
					break;
				}
			}
			goto WAITCMD;
		}

#if EN_WDG
		watchdog_set_wdsv_count(watchdog_calc_timeout_from_sec(WDGTIMEOUT));
#endif
//		delay_s(5);

	} // main loop
}
