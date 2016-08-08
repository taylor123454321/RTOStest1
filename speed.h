/*
 * speed.h
 *
 *  Created on: 28/08/2015
 *      Author: Ryan Taylor
 */

#ifndef SPEED_H_
#define SPEED_H_

#include "init.h"

//float read_speed(void);

int set_speed_func(int set_speed, button_data_s button_data, int screen, float speed);

//int set_speed(int set_speed);

//bool init_set_speed(void);

circBuf_t store_speed(float single_speed, circBuf_t speed_buffer);

float analysis_speed(circBuf_t speed_buffer);

int speed_feedback(float speed, int encoder, int set_speed);

#endif
