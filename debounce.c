/*
 * debounce.c
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#include <stdint.h>
#include <stdbool.h>
#include "debounce.h"
#include "init.h"


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
















