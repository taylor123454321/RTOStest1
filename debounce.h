/*
 * debounce.h
 *
 *  Created on: 29/08/2015
 *      Author: Ryan Taylor
 */

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"


bool button_choker(bool raw, bool raw_p, bool debounced);

void debounce_button(void);

int return_button(void);

bool bit_check(int button_data, int n);



#endif /* DEBOUNCE_H_ */
