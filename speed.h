/*
 * speed.h
 *
 *  Created on: 28/08/2015
 *      Author: Ryan Taylor
 */

#ifndef SPEED_H_
#define SPEED_H_

//char uart_reading(void);

float read_speed(void);

int set_speed(int set_speed);

bool init_set_speed(void);

int speed_feedback(float speed, int encoder, int set_speed);

#endif
