/*
 * debounce.c
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#include <stdint.h>
#include <stdbool.h>
#include "debounce.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"



#define DEBOUNCE_TIME 1000000
#define MAX_24BIT_VAL 0X0FFFFFF


bool DebUpBut = 1;			// 'Official' debounced value for the down button
bool DebDownBut = 1;		// Debounced value for up button
bool DebLeftBut = 1;
bool DebCwBut = 1;
bool DebSelectBut = 1;

bool raw_down = 1;			// raw pin value for the down button
bool raw_up = 1;			// raw pin value for the up button
bool raw_left = 1;
bool raw_cw = 1;
bool raw_select = 1;

bool raw_down_p = 1;		// pin value for previous raw value of down button
bool raw_up_p = 1;			// pin value for previous raw value of up button
bool raw_left_p = 1;
bool raw_cw_p = 1;
bool raw_select_p = 1;

int last_debounce = 0;		// Initialised value for the time of the previous debounce
bool button_pressed = 0;	// variable used to latch button presses


bool button_choker(bool raw, bool raw_p, bool debounced){
	if(raw != raw_p){
		debounced = raw;
		if (debounced == 1)
			button_pressed = 1;
	}
	return debounced;
}

// This function is used for debouncing the buttons
void debounce_button(void){
	int current_time = SysTickValueGet();
	int diff = 0;
	if (current_time <= last_debounce){
		diff = last_debounce - current_time;
	}
	else {
		diff = (last_debounce + MAX_24BIT_VAL) - current_time;
	}
	if(diff > DEBOUNCE_TIME){
		raw_up = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_3) == GPIO_PIN_3);
		raw_down = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_4) == GPIO_PIN_4);
		raw_left = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_5) == GPIO_PIN_5);
		raw_cw = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_6) == GPIO_PIN_6);
		raw_select = (GPIOPinRead (GPIO_PORTG_BASE, GPIO_PIN_7) == GPIO_PIN_7);

		DebUpBut = button_choker(raw_up, raw_up_p, DebUpBut);			// 'Official' debounced value for the down button
		DebDownBut = button_choker(raw_down, raw_down_p, DebDownBut);		// Debounced value for up button
		DebLeftBut = button_choker(raw_left, raw_left_p, DebLeftBut);
		DebCwBut = button_choker(raw_cw, raw_cw_p, DebCwBut);
		DebSelectBut = button_choker(raw_select, raw_select_p, DebSelectBut);

		raw_up_p = raw_up;
 		raw_down_p = raw_down;
		raw_left_p = raw_left;
		raw_cw_p = raw_cw;
		raw_select_p = raw_select;

		last_debounce = current_time;
	}
}


int return_button(void){
	int button_data = 0;
	/*
	* BUTTON_DATA
	* 0 bit | DOWN
	* 1 bit | UP
	* 2 bit | LEFT
	* 3 bit | RIGHT
	* 4 bit | SELECT
	* 5 bit |
	*
	*/
	button_data ^= (-!DebDownBut ^ button_data) & (1 << 0);   // 0 bit | DOWN
	button_data ^= (-!DebUpBut ^ button_data) & (1 << 1);     // 1 bit | UP
	button_data ^= (-!DebLeftBut ^ button_data) & (1 << 2);   // 2 bit | LEFT
	button_data ^= (-!DebCwBut ^ button_data) & (1 << 3);     // 3 bit | RIGHT
	button_data ^= (-!DebSelectBut ^ button_data) & (1 << 4); // 4 bit | SELECT
	return button_data;
}

bool bit_check(int button_data, int n){
	int bit = button_data & (1 << n);
	if (bit == 1 && n == 0){//down
		return 1;
	}
	if (bit == 2 && n == 1){//up
			return 1;
	}
	if (bit == 4 && n == 2){//left
			return 1;
	}
	if (bit == 8 && n == 3){//right
			return 1;
	}
	if (bit == 16 && n == 4){//Select
			return 1;
	}
	return 0;
}




















