/*
 * speed.c
 *
 *  Created on: 28/08/2015
 *      Author: Ryan Taylor
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <RTOStest1/button_data.h>
#include <stdint.h>
#include "speed.h"
#include "display.h"
#include "driverlib/uart.h"
#include "inc/hw_uart.h"
#include "inc/hw_memmap.h"
#include "data_process.h"
#include "init.h"





#define PI 3.14159265358979323846
#define BUF_SIZE 8


set_speed_s set_speed_func(set_speed_s set_speed, button_data_s button_data, float speed){
	if (set_speed.is_speed_set == 1){
		set_speed.set_speed_value = speed;
		set_speed.is_speed_set = 0;
	}
	else {
		if (button_data.up == 1){
			if (set_speed.set_speed_value >= 130){// over 130 it will be capped
				set_speed.set_speed_value = 130;
			}
			else if (set_speed.set_speed_value <= 50){// less then 50 is capped
				set_speed.set_speed_value = 50;
			}
			else{
				set_speed.set_speed_value ++;
			}
		}
		else if (button_data.down == 1){
			if (set_speed.set_speed_value <= 50){// less then 50 is capped
				set_speed.set_speed_value = 50;
			}
			else if (set_speed.set_speed_value >= 130){// over 130 it will be capped
				set_speed.set_speed_value = 130;
			}
			else {
				set_speed.set_speed_value --;
			}
		}
	}
	return set_speed;
}



// This function stores the altitude in a buffer "speed_buffer" for analysis later
circBuf_t store_speed(float single_speed, circBuf_t speed_buffer){
	if (single_speed > 150 ){
		single_speed = 150;
	}
	speed_buffer.data[speed_buffer.windex] = single_speed;
	speed_buffer.windex ++;
	if (speed_buffer.windex >= speed_buffer.size){
		speed_buffer.windex = 0;
	}
	return speed_buffer;
}

// Store speed in buffer
float analysis_speed(circBuf_t speed_buffer){
	int i = 0;
	float speed_sum = 0;
	for (i = 0; i < BUF_SIZE; i++)
	speed_sum += speed_buffer.data[i];
	return (speed_sum/BUF_SIZE);
}

encoder_s encoder_quad(encoder_s encoder, encoder_raw_DATA_s encoder_raw_DATA){
	int current_state = 0;
	if (!encoder_raw_DATA.ul_A_Val){ //Check what state the pins at and assign that state to "current state"
		if(!encoder_raw_DATA.ul_B_Val){
			current_state = 1;
		}
		else{
			current_state = 2;
		}
	}
	else{
		if(encoder_raw_DATA.ul_B_Val){
			current_state = 3;
		}
		else{
			current_state = 4;
		}
	}
	// Check if the previous state is different from the current state.
	// Determine what direction the encoder is spinning
	if (current_state != encoder.prev_state){
		if (abs(encoder.prev_state - current_state) == 1){
			if(current_state > encoder.prev_state){
				encoder.angle --;
			}
			else{
				encoder.angle ++;
			}
		}
		else{
			if(current_state < encoder.prev_state){
				encoder.angle --;
			}
			else{
				encoder.angle ++;
			}
		}
	}
	encoder.prev_state = current_state; // Assign current state for next time the interrupt runs
	return encoder;
}




/*void acceleration_test(float speed, acc_time_s acc_times){
	int current_acc_time = 0;
	bool started = 0;
	if (speed < 3){
		current_acc_time = 0;
		init_acc_time(&acc_times); //reset time
	}
	if (speed > 3 && started == 0) {
		started = 1;
	}
	if (speed > 20 && acc_times.acc20 == 0){
		acc_times.acc20 = current_acc_time;
	}
	if (speed > 40 && acc_times.acc40 == 0){
		acc_times.acc40 = current_acc_time;
	}
	if (speed > 60 && acc_times.acc60 == 0){
		acc_times.acc60 = current_acc_time;
	}
	if (speed > 80 && acc_times.acc80 == 0){
		acc_times.acc80 = current_acc_time;
	}
	if (speed > 100 && acc_times.acc100 == 0) {
		acc_times.acc100 = current_acc_time;
		started = 0;			 //finished
	}
}*/

