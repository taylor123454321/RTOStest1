/*
 * debounce.c
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#include <stdint.h>
#include <stdbool.h>
#include "debounce.h"


bool invert_button(bool button){
	if (button == 0){
		button = 1;
	}
	else {
		button = 0;
	}
	return button;
}















