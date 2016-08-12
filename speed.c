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
#include <stdint.h>
#include "speed.h"
#include "debounce.h"
#include "display.h"
#include "driverlib/uart.h"
#include "inc/hw_uart.h"
#include "inc/hw_memmap.h"
#include "data_process.h"
#include "init.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"




#define PI 3.14159265358979323846
#define BUF_SIZE 8
#define PWM_MIN_DUTY 14


#define GPIOHigh(x) GPIOPinWrite(GPIO_PORTF_BASE, x, x)//GPIO_PIN_1
#define GPIOLow(x) GPIOPinWrite(GPIO_PORTF_BASE, x, 0)

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

encoder_s encoder_quad(encoder_s encoder, unsigned long ul_A_Val, unsigned long ul_B_Val){
	int current_state = 0;
	if (!ul_A_Val){ //Check what state the pins at and assign that state to "current state"
		if(!ul_B_Val){
			current_state = 1;
		}
		else{
			current_state = 2;
		}
	}
	else{
		if(ul_B_Val){
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


int find_dir(int aim_pos){
	int direction = 0;
	if (aim_pos > 0){
		direction = 1;//CCW
	}
	else if (aim_pos < 0){
		direction = 2;//CW
	}
	else{
		direction = 0;//Stopped
	}
	return direction;
}

// this function connects speed to carb/rpm
PWM_DATA_s speed_feedback(float speed, int encoder, int set_speed, PWM_DATA_s PWM_DATA){
	int aim_pos = 0;// this is the position the motor goes to
	int error_speed = set_speed - speed;
	int error_rotation = encoder;
	aim_pos = error_rotation + 100*error_speed;


	if (aim_pos > 98 || aim_pos < -98){
		PWM_DATA.duty = 98;
		PWM_DATA.direction = find_dir(aim_pos);
	}
	else {
		PWM_DATA.duty  = abs(aim_pos) - PWM_MIN_DUTY;
		PWM_DATA.direction = find_dir(aim_pos);
	}

	signed int check = PWM_DATA.duty;

	if (check < PWM_MIN_DUTY && check > -PWM_MIN_DUTY){
		PWM_DATA.duty = 0;
		PWM_DATA.direction = 0;
	}
	return PWM_DATA;
}


void PWM_duty(PWM_DATA_s PWM_DATA, unsigned long period){
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * PWM_DATA.duty / 100);
}

void PWM_direction(PWM_DATA_s PWM_DATA){
	if (PWM_DATA.direction == 1){ // turns motor CCW
		GPIOHigh(GPIO_PIN_3);
		GPIOLow(GPIO_PIN_1);

	}
	else if (PWM_DATA.direction == 2){// turns motor CW
		GPIOHigh(GPIO_PIN_1);
		GPIOLow(GPIO_PIN_3);
	}
	else {							// turns motor off
		GPIOLow(GPIO_PIN_1);
		GPIOLow(GPIO_PIN_3);
	}
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

