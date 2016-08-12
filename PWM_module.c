/*
 * PWM_module.h
 *
 *  Created on: 13/07/2016
 *      Author: Ryan Taylor
 */
//INCLUDEDS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "init.h"

#define PWM_MIN_DUTY 14


#define GPIOHigh(x) GPIOPinWrite(GPIO_PORTF_BASE, x, x)
#define GPIOLow(x) GPIOPinWrite(GPIO_PORTF_BASE, x, 0)


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
PWM_DATA_s speed_feedback(PWM_speed_DATA_s PWM_speed_DATA, encoder_s encoder_1, PWM_DATA_s PWM_DATA){
	int aim_pos = 0;// this is the position the motor goes to
	int error_speed = PWM_speed_DATA.set_speed - PWM_speed_DATA.speed;
	int error_rotation = encoder_1.angle;
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
