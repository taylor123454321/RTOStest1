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
void initPWMchan (void);

void initConsole (void);

void initTimer(void);

void main_init(void);

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
}acc_time_s;

//*****************************************************************************
// Buffer type declaration - set of unsigned longs
//*****************************************************************************
typedef struct {
	unsigned int size;	    // Number of entries in buffer
	unsigned int windex;	// index for writing, mod(size)
	unsigned int rindex;	// index for reading, mod(size)
	float *data;	// pointer to the data
} circBuf_t;


// Data from GPS feed into decoder, this is the struct for that data
typedef struct {
	clock real_time_s;
	long_lat location_s;
	uint8_t fixquality_s;
	uint8_t satellites_s;
	float HDOP_s;
	float altitude_s;
	float geoidheight_s;
	bool fix_s;
	float angle_s;
	float speed_s;
} GPS_DATA_DECODED_s;

typedef struct{
	bool left;
	bool right;
	bool up;
	bool down;
	bool select;
} button_data_s;

typedef struct{
	bool left;
	bool right;
	bool up;
	bool down;
	bool select;
} button_data_raw_s;

typedef struct{
	int prev_state;
	int angle;
} encoder_s;

typedef struct{
	unsigned long duty;
	int direction;
} PWM_DATA_s;

typedef struct{
	int set_speed_value;
	bool is_speed_set;
	int screen;
} set_speed_s;

typedef struct{
	float speed;
	int set_speed;
} PWM_speed_DATA_s;

typedef struct{
	unsigned long ul_A_Val;
	unsigned long ul_B_Val;
} encoder_raw_DATA_s;

/*typedef struct {
	int index;
	char UART_char_data[90];
	char UART_char_data_old[90];

} GPS_DATA_s;*/


// *******************************************************
// initCircBuf: Initialise the circBuf instance. Reset both indices to
// the start of the buffer.  Dynamically allocate and clear the the
// memory and return a pointer for the data.  Return NULL if
// allocation fails.
// *******************************************************
float * initCircBuf (circBuf_t *buffer, unsigned int size);

void init_set_speed_data(cruise_data *data);

void init_acc_time(acc_time_s *data);

#endif /* INIT_H_ */
