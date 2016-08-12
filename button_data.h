/*
 * debounce.h
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#ifndef BUTTON_DATA_H_
#define BUTTON_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "init.h"

button_data_raw_s read_buttons(void);

bool invert_bool(bool button);

button_data_s invert_button(button_data_raw_s raw_button_data);

#endif /* BUTTON_DATA_H_ */
