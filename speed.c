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

#define PI 3.14159265358979323846

bool flag1;
bool flag2;

/*char uart_reading(void){
	char string = 0;
	string = UARTCharGet(UART0_BASE);
	while(string != '$'){
		string = UARTCharGet(UART0_BASE);
		UARTCharPut(UART0_BASE, string);
	}
	return string;
}*/

float read_speed(void){
	float speed = 0;
	speed = get_speed();
	speed = speed*1.852;
	return speed;
}

int set_speed(int set_speed){
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
}


// this function connects speed to carb/rpm
int speed_feedback(float speed, int encoder, int set_speed){
	int aim_pos = 0;// this is the position the stepper goes to
	int error = set_speed - (int)speed;
	aim_pos = 1*error;

	return aim_pos;
}





