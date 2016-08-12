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



//float calculate_distance(void);

//float calculate_accleration(void);

//void calculate_speed_future(float a);

GPS_DATA_DECODED_s decode_GGA(char *p, GPS_DATA_DECODED_s DATA);

GPS_DATA_DECODED_s decode_RMC(char *p, GPS_DATA_DECODED_s DATA);

//void update_array(void);

//void Timer0IntHandler(void);

//GPS_DATA_DECODED_s restructure_data(GGA_DATA_s GGA_DATA, RMC_DATA_s RMC_DATA);

GPS_DATA_DECODED_s split_data(char *data_incoming, GPS_DATA_DECODED_s GPS_DATA_DECODED);





#endif /* DATA_PROCESS_H_ */
