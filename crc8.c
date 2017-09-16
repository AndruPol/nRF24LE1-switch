/*
 * crc8.c
 *
 *  Created on: 20/09/2016
 *      Author: andru
 */

#include <stdint.h>

#include "crc8.h"

#if defined(USECRC8TBL) && USECRC8TBL

// CRC8 data source table
static const uint8_t dscrc_table[] = {
  0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
  35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
  70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
  17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
  50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
  87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

uint8_t CRC8(uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = dscrc_table[crc ^ *addr++];
	}
	return crc;
}

#else

static uint8_t crcDS(uint8_t inp, uint8_t crc)
{
	inp ^= crc;
	crc = 0;
	if(inp & 0x1)   crc ^= 0x5e;
	if(inp & 0x2)   crc ^= 0xbc;
	if(inp & 0x4)   crc ^= 0x61;
	if(inp & 0x8)   crc ^= 0xc2;
	if(inp & 0x10)  crc ^= 0x9d;
	if(inp & 0x20)  crc ^= 0x23;
	if(inp & 0x40)  crc ^= 0x46;
	if(inp & 0x80)  crc ^= 0x8c;
	return crc;
}

uint8_t CRC8(uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	while (len--) {
		crc = crcDS(*addr++, crc);
	}
	return crc;
}

#endif

