/*
 * data_process.h
 *
 *  Created on: 30/08/2015
 *      Author: Ryan Taylor
 */

#ifndef DATA_PROCESS_H_
#define DATA_PROCESS_H_

#include <stdint.h>
#include "init.h"
#include "driverlib/timer.h"

float get_speed(void);

clock read_time(void);

uint8_t read_satillite(void);

long_lat read_location(void);

bool read_fix(void);

float read_quality(void);

float read_distance(void);

float calculate_distance(void);

float calculate_distance(void);

float read_acceleration(float buff_s);

float calculate_accleration(void);

//void calculate_speed_future(float a);

GGA_DATA_s decode_GGA(char *p);

RMC_DATA decode_RMC(char *p);

void update_array(void);

//void Timer0IntHandler(void);

GPS_DATA_DECODED_s split_data(char *data_incoming);


typedef struct {
	clock real_time_s;
	long_lat location_s;
} GGA_DATA_s;

typedef struct {
	long UART_character;
} RMC_DATA_s;

typedef struct {
	GGA_DATA_s firstbit;
	RMC_DATA_s secondbit;
} GPS_DATA_DECODED_s;

GGA_DATA_s
RMC_DATA_s
GPS_DATA_DECODED_s


#endif /* DATA_PROCESS_H_ */
