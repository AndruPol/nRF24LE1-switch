/*
 * main.h
 *
 *  Created on: 15.04.2017
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#define DEBUG	0	// debug output via UART
#define printf	printf_tiny

#define EN_LED	1		// LED enable
#define EN_RF	1		// radio enable
#define EN_RTC	1		// RTC for send status enable
#define EN_WDG	1		// watchdog enable
#define EN_SW	1		// external switch enable
#define EN_AES	1		// use AES
#define MSGLEN	sizeof(MESSAGE_T)

#include <stdint.h>

// NVM configuration data for nRF24LE1, CRC8 must be next byte
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint8_t magic;		// magic byte
	uint8_t version;	// configuration version
	uint8_t deviceID;	// device ID
	uint8_t channel;	// radio channel: 0-199
	uint8_t datarate;	// data rate: 1-3
	uint8_t autoask;	// auto ask
	uint8_t srvaddr[5];	// gateway address to send
	uint8_t devaddr[5];	// device address to receive command
#if EN_AES
	uint8_t useaes;		// use aes encryption
	uint8_t aeskey[16];	// aes encryption key
#endif
	uint8_t maxsend;	// max message send retries
	uint8_t enable;		// switch enable
	uint16_t report;	// time interval (S) to send report
};

typedef enum {
	ADDR_SWITCH = 0,	// switch output
	ADDR_BUTTON,		// button input
	CFG_MAXSEND,		// uint8_t maxsend;
	CFG_ENABLE,			// uint8_t enable;
	CFG_REPORT,			// uint16_t report;
} address_t;

typedef enum {
	SWITCH_OK,
	SWITCH_PARAM,		// parameter error
	SWITCH_DISABLE,		// switch disabled
} switch_error_t;

typedef enum {
	SENSOR_INFO = 0,
	SENSOR_DATA,
	SENSOR_ERROR,
	SENSOR_CMD,
} msgtype_t;

typedef enum {
	DS1820 = 0,
	BH1750,
	DHT,
	BMP085,
	ADC,
	HCSR04,
} sensortype_t;

typedef enum {
	TEMPERATURE = 0,
	HUMIDITY,
	PRESSURE,
	LIGHT,
	VOLTAGE,
	DISTANCE,
} valuetype_t;

typedef enum {
	CMD_CFGREAD = 1,	// read configuration value
	CMD_CFGWRITE,		// write configuration value
	CMD_RESET,			// reset device
	CMD_SENSREAD = 10,	// read sensor value
	CMD_ON = 20,		// ON
	CMD_OFF,			// OFF
	CMD_ONTM,			// ON timeout (S) message.data.iValue
	CMD_OFFTM,			// OFF timeout (S) message.data.iValue
} command_t;

// radio message structure = 16 bytes length
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE {
	msgtype_t msgType;			// message type
	uint8_t deviceID;			// remote device ID
	sensortype_t sensorType;	// sensor type
	valuetype_t valueType;		// value type
	address_t address;			// internal sensor address
	command_t command;			// command
	uint8_t error;				// error code
	uint8_t firmware;			// firmware version
	uint8_t notused[4];			//
	union	{					// sensor value depend on sensor type
		float	fValue;			// float value
		int32_t	iValue;			// integer value/command parameter
		uint8_t cValue[4]; 		//
	} data;
};

typedef enum {
	VBAT_OK = 0,				// not used
	VBAT_LOW,
} vbaterr_t;

typedef enum {
	CFGSET_OK = 0,
	CFGSET_WRITE,		// config write error
	CFGSET_PARAM,		// config input param error
} cfgset_t;

extern CONFIG_T config;

#endif /* MAIN_H_ */
