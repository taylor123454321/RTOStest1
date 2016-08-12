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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"



#define false 0
#define true 1

#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n" // Frequency 5Hz
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n" // Frequency 10Hz
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"// turn on GPRMC and GGA
// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C\r\n"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D\r\n"


// UART Send data, used in conjuction with send_data()
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount, int type) {
    //
    // Loop while there are more characters to send.
    //
	long base = 0;
	if(type == 0){
		base = UART0_BASE;
	}
	else if(type == 1){
		base = UART1_BASE;
	}
    while(ulCount--) {
        //
        // Write the next character to the UART.
        //
        //UARTCharPutNonBlocking(base, *pucBuffer++);
    	UARTCharPut(base, *pucBuffer++);
    }
}

// Send instruction to the GPS to set up how to frequency and inputs
void send_data(void){
	int i = 0;
	while(i <= 100000){i++;}
	i = 0;
	UARTSend((unsigned char *)PMTK_SET_NMEA_UPDATE_5HZ, 18, 0); // Set frequency

	while(i <= 100000){i++;}
	i = 0;
	UARTSend((unsigned char *)PMTK_SET_NMEA_OUTPUT_RMCGGA, 53,0); // Set receaving for only RMC and GGA

	while(i <= 1000000){i++;}
	UARTSend((unsigned char *)PGCMD_NOANTENNA, 16,0); // Set no antenna
}


// This function reads the value from ADC0 and returns it
unsigned long run_adc(void){
	uint16_t uiValue = 10;
	unsigned long ulValue[1];
	ADCProcessorTrigger(ADC0_BASE, 3);
	//
	// Wait for conversion to be completed.
	while(!ADCIntStatus(ADC0_BASE, 3, false))
	{
	}
	// Read ADC Value.
	ADCSequenceDataGet(ADC0_BASE, 3, ulValue);
	uiValue = (unsigned int) ulValue[0];

	return uiValue;
}
