/*
 * speed.h
 *
 *  Created on: 28/08/2015
 *      Author: Ryan Taylor
 */

#ifndef SPEED_H_
#define SPEED_H_

#include "init.h"

set_speed_s set_speed_func(set_speed_s set_speed, button_data_s button_data, float speed);

circBuf_t store_speed(float single_speed, circBuf_t speed_buffer);

float analysis_speed(circBuf_t speed_buffer);

encoder_s encoder_quad(encoder_s encoder, unsigned long ul_A_Val, unsigned long ul_B_Val);

int find_dir(int aim_pos);

PWM_DATA_s speed_feedback(PWM_speed_DATA_s PWM_speed_DATA, int encoder, PWM_DATA_s PWM_DATA);

void PWM_duty(PWM_DATA_s PWM_DATA, unsigned long period);

void PWM_direction(PWM_DATA_s PWM_DATA);

//void acceleration_test(float speed, acc_time_s acc_times);


#endif
