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

void password_display(void);

void init_password(void);

void set_speed_display(int speed_set);

void enter_fuel_display(void);

void review_fuel_display(void);

void no_fix_screen(clock time, uint8_t satellite, int encoder, int aim_pos);

void accleration_screen(float speed, float acc, float max_acc, acc_time acc_times);

void display(int screen, float speed, float acc, float max_acc, int speed_set, uint8_t satellite,
		int encoder_3, clock time, float distance, float quality, char * stuff, int aim_pos, unsigned long adc, acc_time acc_times);


#endif /* DISPLAY_H_ */
