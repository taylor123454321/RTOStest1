/*
 * debounce.c
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#include <RTOStest1/button_data.h>
#include <stdint.h>
#include <stdbool.h>
#include "init.h"
#include "driverlib/gpio.h"
#include "inc\hw_memmap.h"


button_data_raw_s read_buttons(void){
	button_data_raw_s raw_button_data;
	raw_button_data.up = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_3) == GPIO_PIN_3);
	raw_button_data.down = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_4) == GPIO_PIN_4);
	raw_button_data.left = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_5) == GPIO_PIN_5);
	raw_button_data.right = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_6) == GPIO_PIN_6);
	raw_button_data.select = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_7) == GPIO_PIN_7);
	return raw_button_data;
}

bool invert_bool(bool button){
	if (button == 0){
			button = 1;
		}
		else {
			button = 0;
		}
	return button;
}


button_data_s invert_button(button_data_raw_s raw_button_data){
	button_data_s button_data;
	button_data.down = invert_bool(raw_button_data.down);
	button_data.left = invert_bool(raw_button_data.left);
	button_data.right = invert_bool(raw_button_data.right);
	button_data.select = invert_bool(raw_button_data.select);
	button_data.up = invert_bool(raw_button_data.up);
	return button_data;
}
















