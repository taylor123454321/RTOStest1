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


#define GPIOHigh(x) GPIOPinWrite(GPIO_PORTF_BASE, x, x)//GPIO_PIN_1
#define GPIOLow(x) GPIOPinWrite(GPIO_PORTF_BASE, x, 0)

bool flag1;
bool flag2;


/*float read_speed(void){
	float speed = 0;
	speed = get_speed();
	speed = speed*1.852;
	return speed;
}*/

int set_speed_func(int set_speed, button_data_s button_data, int screen, float speed){
	if (screen == 2){
		set_speed = speed;
	}
	else {
		if (button_data.up == 1){
			set_speed ++;
		}
		else if (button_data.down == 1){
			set_speed --;
		}
	}
	return set_speed;
}

/*int set_speed(int set_speed){
	int button_data = return_button();
	bool down = bit_check(button_data, 0);
	bool up = bit_check(button_data, 1);

	//bool left = button_data & (1 << 2);
	//bool right = button_data & (1 << 3);
	//bool select = button_data & (1 << 4);

	if(flag2 == 0){
		flag2 = 1;
	}

	if (up == 1 && set_speed < 140 && flag1 == 1){
		if (set_speed > 130){// over 130 it will be capped
			set_speed = 130;
		}
		else if (set_speed < 50){// less then 50 is capped
			set_speed = 50;
		}
		else{
			set_speed ++;
		}
		flag1 = 0;
	}
	else if(down == 1 && set_speed > 0 && flag1 == 1){
		if (set_speed < 50){// less then 50 is capped
			set_speed = 50;
		}
		else if (set_speed > 130){// over 130 it will be capped
			set_speed = 130;
		}
		else {
			set_speed --;
		}
		flag1 = 0;
		if (set_speed == 99){
			clearDisplay();
		}
	}
	else if (down == 0 && up == 0){
		flag1 = 1;
	}
	return set_speed;
}

bool init_set_speed(void){
	return flag2;
}*/


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
				encoder.encoder --;
			}
			else{
				encoder.encoder ++;
			}
		}
		else{
			if(current_state < encoder.prev_state){
				encoder.encoder --;
			}
			else{
				encoder.encoder ++;
			}
		}
	}
	encoder.prev_state = current_state; // Assign current state for next time the interrupt runs
	return encoder;
}


// this function connects speed to carb/rpm
PWM_DATA_s speed_feedback(float speed, int encoder, int set_speed, PWM_DATA_s PWM_DATA){
	int aim_pos = 0;// this is the position the stepper goes to
	int error = set_speed - (encoder/100);
	aim_pos = 1*error;
	if (aim_pos < 0){
		PWM_DATA.direction = 0;
		aim_pos = aim_pos;
	}
	else if (aim_pos >= 0){
		PWM_DATA.direction = 1;
		aim_pos = aim_pos;

	}
	aim_pos = abs(aim_pos);
	if (aim_pos < 13){
		aim_pos = 0;
	}
	else if (aim_pos > 98){
		aim_pos = 98;
	}
	PWM_DATA.duty = aim_pos;

	return PWM_DATA;
}


void PWM_duty(PWM_DATA_s PWM_DATA, unsigned long period){
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * PWM_DATA.duty / 100);
}

void PWM_direction(PWM_DATA_s PWM_DATA){
	if (PWM_DATA.direction == 1){
		GPIOHigh(GPIO_PIN_3);
		GPIOLow(GPIO_PIN_2);

	}
	else if (PWM_DATA.direction == 0){
		GPIOHigh(GPIO_PIN_3);
		GPIOLow(GPIO_PIN_1);
	}
	else {
		GPIOLow(GPIO_PIN_1);
		GPIOLow(GPIO_PIN_2);
	}
}



