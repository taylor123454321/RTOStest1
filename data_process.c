/*
 * data_process.c
 *
 *  Created on: 30/08/2015
 *      Author: Ryan Taylor
 */
//INCLUDEDS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "display.h"
#include "init.h"
#include "speed.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "data_process.h"


#define false 0
#define true 1
#define MAX_24BIT_VAL 0X0FFFFFF
//GLOABAL VARIABLES

//float buffed_speed = 0;
float buffed_speed_1_old = 0;
float acceleration = 0;
float distance = 0;

float time_array[5] = {1, 2, 3, 4, 5};
float speed_array[5];

int last_time = 0;		// Initialised value for the time of the previous debounce

//FUNCTIONS


/*float calculate_distance(void){
	int current_time = SysTickValueGet();
	int delta_time = 0;
	if (current_time <= last_time){
		delta_time = last_time - current_time;
	}
	else {
		delta_time = (last_time + MAX_24BIT_VAL) - current_time;
	}

	float distance = (buffed_speed/3.6)*(delta_time/10000000);

	last_time = current_time;

	if (distance > 1){
		return distance;
	}
	return 0;
}*/


/*float calculate_accleration(void){
	float delta = buffed_speed - buffed_speed_1_old;
	if (delta > 0.5 || delta < -0.5){
		return delta;
	}
	return 0;
}*/

/*void calculate_speed_future(float a){
    float s = 1, t = 1, k = 0;// variables used in data
    int n = 5;
    int i = 0;
    int j = 0;

    // a is enter time value we want to find
    for(i = 0; i < n; i++) {
         s = 1;
         t = 1;
         for(j = 0; j < n; j++) {
              if(j != i){
                  s = s*(a - time_array[j]);
                  t = t*(time_array[i] - time_array[j]);
              }
          }
          k = k + ((s/t)*speed_array[i]); // k is final
    }

    speed_guessed = k;
}*/

// Decode fucntion for GGA NEMA sentence
GPS_DATA_DECODED_s decode_GGA(char *p, GPS_DATA_DECODED_s DATA){
	int32_t degree;
	long minutes;
	char degreebuff[10];
	clock real_time;
	long_lat location;
	float latitude = 0;
	float longitude = 0;
	uint32_t latitudeDegrees = 0;
	uint32_t longitudeDegrees = 0;
	char lat, lon;


	// GET TIME
	p = strchr(p, ',')+1;
	float timef = atof(p);
	uint32_t time = timef;
	real_time.hour = time / 10000;
	real_time.minute = (time % 10000) / 100;
	real_time.seconds = (time % 100);
	real_time.milliseconds = fmod(timef, 1.0) * 1000;
	DATA.real_time_s = real_time;

	// GET LATITUDE
	p = strchr(p, ',')+1;
	if (',' != *p) {
		strncpy(degreebuff, p, 2);
		p += 2;
		degreebuff[2] = '\0';
		degree = atol(degreebuff) * 10000000;
		strncpy(degreebuff, p, 2); // minutes
		p += 3; // skip decimal point
		strncpy(degreebuff + 2, p, 4);
		degreebuff[6] = '\0';
		minutes = 50 * atol(degreebuff) / 3;
		location.latitude_s = degree + minutes;
		latitude = degree / 100000 + minutes * 0.000006F;
		int convert = latitude/100;
		latitudeDegrees = (latitude - 100 * convert)/60.0 ;
		latitudeDegrees += convert;
	}

	p = strchr(p, ',')+1;
	if (',' != *p) {
		if (p[0] == 'S') latitudeDegrees *= -1.0;
	    if (p[0] == 'N') lat = 'N';
	    else if (p[0] == 'S') lat = 'S';
	    else if (p[0] == ',') lat = 0;
	    //else return false;
	}

	// GET LONGDITUDE
	p = strchr(p, ',')+1;
	if (',' != *p) {
		strncpy(degreebuff, p, 3);
	    p += 3;
	    degreebuff[3] = '\0';
	    degree = atol(degreebuff) * 10000000;
	    strncpy(degreebuff, p, 2); // minutes
	    p += 3; // skip decimal point
	    strncpy(degreebuff + 2, p, 4);
	    degreebuff[6] = '\0';
	    minutes = 50 * atol(degreebuff) / 3;
	    location.longitude_s = degree + minutes;
	    longitude = degree / 100000 + minutes * 0.000006F;
	    int convert_2 = longitude/100;
	    longitudeDegrees = (longitude - 100*convert_2)/60.0;
	    longitudeDegrees += convert_2;
	}
	DATA.location_s = location;

	p = strchr(p, ',')+1;
	if (',' != *p){
		if (p[0] == 'W') longitudeDegrees *= -1.0;
	    if (p[0] == 'W') lon = 'W';
	    else if (p[0] == 'E') lon = 'E';
	    else if (p[0] == ',') lon = 0;
	    //else return false;
	}
	p = strchr(p, ',')+1;
	if (',' != *p){
		DATA.fixquality_s = atoi(p);
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
		DATA.satellites_s = atoi(p);
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
	    DATA.HDOP_s = atof(p);
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
	    DATA.altitude_s = atof(p);
	}

	p = strchr(p, ',')+1;
	p = strchr(p, ',')+1;
	if (',' != *p){
	    DATA.geoidheight_s = atof(p);
	}
	return DATA;
}

// Decode fucntion for RMC NEMA sentence
GPS_DATA_DECODED_s decode_RMC(char *p, GPS_DATA_DECODED_s DATA){
	int32_t degree;
	long minutes;
	char degreebuff[10];

	clock real_time;
	float longitude = 0;
	float latitude = 0;
	uint32_t latitudeDegrees = 0;
	uint32_t longitudeDegrees = 0;
	char lat, lon;
	int32_t longitude_fixed = 0;
	int32_t latitude_fixed = 0;
	bool fix = 0;

	// get time
	p = strchr(p, ',')+1;
	float timef = atof(p);
	uint32_t time = timef;
	real_time.hour = time / 10000;
	real_time.minute = (time % 10000) / 100;
	real_time.seconds = (time % 100);
	real_time.milliseconds = fmod(timef, 1.0) * 1000;

	p = strchr(p, ',')+1;
	// Serial.println(p);
	if (p[0] == 'A')
	    fix = true;
	else if (p[0] == 'V')
	    fix = false;
	//else
	//    return false;
	DATA.fix_s = fix;

	// parse out latitude
	p = strchr(p, ',')+1;
	if (',' != *p){
	    strncpy(degreebuff, p, 2);
	    p += 2;
	    degreebuff[2] = '\0';
	    long degree = atol(degreebuff) * 10000000;
	    strncpy(degreebuff, p, 2); // minutes
	    p += 3; // skip decimal point
	    strncpy(degreebuff + 2, p, 4);
	    degreebuff[6] = '\0';
	    long minutes = 50 * atol(degreebuff) / 3;
	    latitude_fixed = degree + minutes;
	    latitude = degree / 100000 + minutes * 0.000006F;
	    int convert = latitude/100;
	    latitudeDegrees = (latitude - 100*convert)/60.0;
	    latitudeDegrees += convert;
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
	    if (p[0] == 'S') latitudeDegrees *= -1.0;
	    if (p[0] == 'N') lat = 'N';
	    else if (p[0] == 'S') lat = 'S';
	    else if (p[0] == ',') lat = 0;
	    //else return false;
	}

	// parse out longitude
	p = strchr(p, ',')+1;
	if (',' != *p){
	    strncpy(degreebuff, p, 3);
	    p += 3;
	    degreebuff[3] = '\0';
	    degree = atol(degreebuff) * 10000000;
	    strncpy(degreebuff, p, 2); // minutes
	    p += 3; // skip decimal point
	    strncpy(degreebuff + 2, p, 4);
	    degreebuff[6] = '\0';
	    minutes = 50 * atol(degreebuff) / 3;
	    longitude_fixed = degree + minutes;
	    longitude = degree / 100000 + minutes * 0.000006F;
	    int convert_2 = longitude/100;
	    longitudeDegrees = (longitude-100*convert_2)/60.0;
	    longitudeDegrees += convert_2;
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
	    if (p[0] == 'W') longitudeDegrees *= -1.0;
	    if (p[0] == 'W') lon = 'W';
	    else if (p[0] == 'E') lon = 'E';
	    else if (p[0] == ',') lon = 0;
	    //else return false;
	}
	// speed
	p = strchr(p, ',')+1;
	if (',' != *p){
		DATA.speed_s = atof(p);
	}

	// angle
	p = strchr(p, ',')+1;
	if (',' != *p){
	    DATA.angle_s = atof(p);
	}

	p = strchr(p, ',')+1;
	if (',' != *p){
	    uint32_t fulldate = atof(p);
	    real_time.day = fulldate / 10000;
	    real_time.month = (fulldate % 10000) / 100;
	    real_time.year = (fulldate % 100);
	}
	return DATA;
}

/*void update_array(void){
	int i;
	for (i = 0; i < 3; i++){
		speed_array[i] = speed_array[i+1];
	}
	speed_array[4] = speed;
}*/

/*void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //update_array();
    distance += calculate_distance();
    acceleration = calculate_accleration();
    buffed_speed_1_old = buffed_speed;
    //IntMasterDisable();
    //IntMasterEnable();
}*/


// Make one struct instead of two
/*GPS_DATA_DECODED_s restructure_data(GGA_DATA_s GGA_DATA, RMC_DATA_s RMC_DATA){
	GPS_DATA_DECODED_s GPS_DATA_DECODED;
	GPS_DATA_DECODED.HDOP_s = GGA_DATA.HDOP_s;
	GPS_DATA_DECODED.altitude_s = GGA_DATA.altitude_s;
	GPS_DATA_DECODED.geoidheight_s = GGA_DATA.geoidheight_s;
	GPS_DATA_DECODED.real_time_s = GGA_DATA.real_time_s;
	GPS_DATA_DECODED.location_s = GGA_DATA.location_s;
	GPS_DATA_DECODED.fixquality_s = GGA_DATA.fixquality_s;
	GPS_DATA_DECODED.satellites_s = GGA_DATA.satellites_s;
	GPS_DATA_DECODED.fix_s = RMC_DATA.fix_s;
	GPS_DATA_DECODED.angle_s = RMC_DATA.angle_s;
	GPS_DATA_DECODED.speed_s = RMC_DATA.speed_s;
	return GPS_DATA_DECODED;
}*/

//Split data up into GGA,GSA,RMC,VTG
GPS_DATA_DECODED_s split_data(char *data_incoming, GPS_DATA_DECODED_s GPS_DATA_DECODED){
	if (strstr(data_incoming, "GGA") != NULL){
		GPS_DATA_DECODED = decode_GGA(data_incoming, GPS_DATA_DECODED);
	}
	else if (strstr(data_incoming, "RMC") != NULL){
		GPS_DATA_DECODED = decode_RMC(data_incoming, GPS_DATA_DECODED);
	}
	else if (strstr(data_incoming, "GSA") != NULL){
	}
	else if (strstr(data_incoming, "VTG") != NULL){
	}

	//GPS_DATA_DECODED = restructure_data(GGA_DATA, RMC_DATA);

	return GPS_DATA_DECODED;
}

















