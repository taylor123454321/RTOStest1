/*
 * init.h
 *
 *  Created on: May 13, 2015
 *      Author: Ryan Taylor
 */

#ifndef INIT_H_
#define INIT_H_

#include <stdint.h>
#include <stdbool.h>

// *******************************************************
// Contstants
// *******************************************************

#define SYSTICK_RATE_HZ 1000ul
#define PWM4_RATE_HZ 150
#define PWM_DIV_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4
#define PWM4_DEF_DUTY 50
#define MAX_24BIT_VAL 0X0FFFFFF

// *******************************************************
// Prototype functions
// *******************************************************

void PinChangeIntHandler (void);

// *******************************************************
// Initial functions
// *******************************************************

// Clears all the peripheral used
void reset_peripheral(void);

// Sets up the proccesor speed and system tick clock
void initClock (void);

void initGPIO(void);

// Sets up the interrupt pins to PF7 and PF5 to be used for the Yaw
void initPin (void);

// Intialise the OLED display
void initDisplay (void);

// Intialise ADC with all the set up conditions for ADC0
void initADC(void);

// Initlise the PWM for pin PWM4. This sets up the period and frequecy also.
//void initPWMchan (void);

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount, int type);

void initConsole (void);

void initTimer(void);

// Strut for time
typedef struct{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
	uint16_t milliseconds;
} clock;

// struct for location
typedef struct{
	int32_t longitude_s;
	int32_t latitude_s;
}long_lat;

// speed data
typedef struct {
	float speed;
	bool enable;
	float old;
}cruise_data;

// acceleration timing data
typedef struct {
	float acc20;
	float acc40;
	float acc60;
	float acc80;
	float acc100;
}acc_time;

//*****************************************************************************
// Buffer type declaration - set of unsigned longs
//*****************************************************************************
typedef struct {
	unsigned int size;	    // Number of entries in buffer
	unsigned int windex;	// index for writing, mod(size)
	unsigned int rindex;	// index for reading, mod(size)
	float *data;	// pointer to the data
} circBuf_t;



// *******************************************************
// initCircBuf: Initialise the circBuf instance. Reset both indices to
// the start of the buffer.  Dynamically allocate and clear the the
// memory and return a pointer for the data.  Return NULL if
// allocation fails.
// *******************************************************
float * initCircBuf (circBuf_t *buffer, unsigned int size);

void init_set_speed_data(cruise_data *data);

void init_acc_time(acc_time *data);

#endif /* INIT_H_ */
