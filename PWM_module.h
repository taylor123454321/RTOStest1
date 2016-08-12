/*
 * PWM_module.h
 *
 *  Created on: 13/07/2016
 *      Author: Ryan Taylor
 */

#ifndef PWM_MODULE_H_
#define PWM_MODULE_H_

#include <stdint.h>
#include "init.h"


int find_dir(int aim_pos);

PWM_DATA_s speed_feedback(PWM_speed_DATA_s PWM_speed_DATA, encoder_s encoder_1, PWM_DATA_s PWM_DATA);

void PWM_duty(PWM_DATA_s PWM_DATA, unsigned long period);

void PWM_direction(PWM_DATA_s PWM_DATA);


#endif /* PWM_MODULE_H_ */
