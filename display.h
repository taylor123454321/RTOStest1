/*
 * display.h
 *
 *  Created on: 28/08/2015
 *      Author: Ryan Taylor
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_


#include <stdint.h>
#include "init.h"

// This function clears the display
void clearDisplay(void);

set_speed_s read_button_screen(button_data_s button_data, set_speed_s set_speed, bool fix);
void password_display(void);

void init_password(void);

void set_speed_display(int speed_set);

void enter_fuel_display(void);

void review_fuel_display(void);

void no_fix_screen(clock time, uint8_t satellite, int encoder, int aim_pos, PWM_DATA_s PWM_DATA);

void accleration_screen(float speed, float acc, float max_acc, acc_time_s acc_times);

void display(int screen, float acc, float max_acc, int speed_set, GPS_DATA_DECODED_s DATA, float buffed_speed,
		unsigned long encoder, float distance, char * stuff, int aim_pos, unsigned long adc,  acc_time_s acc_times, PWM_DATA_s PWM_DATA);

#endif /* DISPLAY_H_ */
