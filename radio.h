/*
 * radio.h
 *
 *  Created on: 10/07/2016
 *      Author: andru
 */

#ifndef RADIO_H_
#define RADIO_H_

void radio_init();
uint8_t rfsend(const MESSAGE_T *msg);
uint8_t rfread(MESSAGE_T *msg, uint16_t timeout);
uint8_t rfreadqueue(MESSAGE_T *msg);
void rfpwrDown(void);

#endif /* RADIO_H_ */
