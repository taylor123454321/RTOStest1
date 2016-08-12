/*
 * data_process.h
 *
 *  Created on: 30/08/2015
 *      Author: Ryan Taylor
 */

#ifndef ADC_INTERFACE_H_
#define ADC_INTERFACE_H_

#include <stdint.h>

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount, int type);

void send_data(void);

unsigned long run_adc(void);


#endif /* DATA_PROCESS_H_ */
