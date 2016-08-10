/*
    FreeRTOS V6.0.5 - Copyright (C) 2009 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* C includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/* FreeRTOS includes. */
#include "include/FreeRTOS.h"
#include "include/task.h"
#include "include/semphr.h"

/* Stellaris library includes. */
#include "inc\hw_types.h"
#include "inc\hw_memmap.h"
//#include "inc\hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "rit128x96x4.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"


#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "inc/hw_uart.h"

/* User made library includes */
#include "init.h"
#include "data_process.h"
#include "debounce.h"
#include "display.h"
#include "speed.h"



/* Demo includes. */
#include "demo_code\basic_io.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )
#define BUF_SIZE 8


/* The task function. */
void vReadGPS( void *pvParameters );
void vDisplayTask( void *pvParameters );
void vFakeGPS( void *pvParameters );
void vDebounceButtons( void *pvParameters );
void vFilterSpeed( void *pvParameters );
//void vEncoder( void *pvParameters );
void vPWM( void *pvParameters );

char UART_char_data[120];
char UART_char_data_old[120];
int index = 0;
int value = 0;
encoder_s encoder_1;
signed int set_speed = 0;
PWM_DATA_s PWM_DATA;



xSemaphoreHandle  xBinarySemaphoreGPS;
xSemaphoreHandle  xBinarySemaphoreFilter;
xSemaphoreHandle  xBinarySemaphoreEncoder_1;
xQueueHandle xQueueGPSDATA;
xQueueHandle xQueueButtons;
xQueueHandle xQueueSpeed;
xQueueHandle xQueueBuffedSpeed;

void store_char(long UART_character, portBASE_TYPE xHigherprioritytaskWoken){
	if (UART_character == '$'){
		UART_char_data_old[0] = '\0';
		strcpy(UART_char_data_old, UART_char_data);
		index = 0;
		//read_data = 0;
		UART_char_data[index] = UART_character;
		index++;
		xSemaphoreGiveFromISR(xBinarySemaphoreGPS, &xHigherprioritytaskWoken);
	}
	else{
		UART_char_data[index] = UART_character;
		index++;
	}
}

void UARTIntHandler(void) {
	portBASE_TYPE xHigherprioritytaskWoken = pdFALSE;
	unsigned long ulStatus;
	long UART_character = 0;
	ulStatus = UARTIntStatus(UART0_BASE, true);	// Get the interrupt status.
	UARTIntClear(UART0_BASE, ulStatus);	    // Clear the asserted interrupts.

	// Loop while there are characters in the receive FIFO.
	while(UARTCharsAvail(UART0_BASE)) {
	    // Read the next character from the UART and write it back to the UART.
	    //UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
	    UART_character = UARTCharGetNonBlocking(UART0_BASE);
	    store_char(UART_character, xHigherprioritytaskWoken);
	}
    portEND_SWITCHING_ISR(xHigherprioritytaskWoken);
}

void PinChangeIntHandler (void){
	unsigned long ul_A_Val;
	unsigned long ul_B_Val;
	//portBASE_TYPE xHigherprioritytaskWoken = pdFALSE;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_7);
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5);

	ul_A_Val = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7); // Read the pin
	ul_B_Val = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);

	value ++;
	encoder_1 = encoder_quad(encoder_1, ul_A_Val, ul_B_Val);

	//xSemaphoreGiveFromISR(xBinarySemaphoreEncoder_1, &xHigherprioritytaskWoken);
    //portEND_SWITCHING_ISR(xHigherprioritytaskWoken);

}

void Timer0IntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);}
void Timer1IntHandler(void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);}
/*-----------------------------------------------------------*/
int main( void ) {
	IntMasterDisable();
	reset_peripheral();
	initClock();
	initPin();
	initTimer();
	initGPIO();
	initConsole();
	initPWMchan();
	send_data();send_data();send_data();

	vSemaphoreCreateBinary(xBinarySemaphoreGPS);
	vSemaphoreCreateBinary(xBinarySemaphoreFilter);
	//vSemaphoreCreateBinary(xBinarySemaphoreEncoder_1);
	//xBinarySemaphoreEncoder_1 = xSemaphoreCreateCounting(50,0);

	xQueueGPSDATA = xQueueCreate( 1, sizeof(GPS_DATA_DECODED_s));
	xQueueButtons = xQueueCreate( 1, sizeof(button_data_s));
	xQueueSpeed = xQueueCreate( 1, sizeof(float));
	xQueueBuffedSpeed = xQueueCreate( 1, sizeof(float));

	/* Create tasks. */
	xTaskCreate( vReadGPS, "GPS Read Task", 240, NULL, 4, NULL );
	xTaskCreate( vDisplayTask, "Display Task", 600, NULL, 1, NULL );
	//xTaskCreate( vFakeGPS, "FakeGPS Task", 300, NULL, 1, NULL );
	xTaskCreate( vDebounceButtons, "Debounce Buttons Task", 240, NULL, 2, NULL );
	xTaskCreate( vFilterSpeed, "Filter Speed Task", 240, NULL, 2, NULL );
	//xTaskCreate( vEncoder, "Encoder Task", 240, NULL, 3, NULL );
	xTaskCreate( vPWM, "PWM Task", 240, NULL, 3, NULL );

	IntMasterEnable();

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();	
	while(1);
}
/*-----------------------------------------------------------*/
void vReadGPS( void *pvParameters ){
	xSemaphoreTake(xBinarySemaphoreGPS, 0);
	GPS_DATA_DECODED_s GPS_DATA_DECODED;
	while(1) {
		GPS_DATA_DECODED = split_data(UART_char_data_old, GPS_DATA_DECODED);
		xQueueSendToBack(xQueueGPSDATA, &GPS_DATA_DECODED, 0);

		xSemaphoreGive(xBinarySemaphoreFilter);
		xQueueSendToBack(xQueueSpeed, &GPS_DATA_DECODED.speed_s, 0);
		xSemaphoreTake(xBinarySemaphoreGPS, portMAX_DELAY);
	}
}

void vDebounceButtons(void *pvParameters){
	button_data_s button_data;
	button_data_raw_s raw_button_data;
	while(1){
		raw_button_data = read_buttons();
		button_data = invert_button(raw_button_data);
		xQueueSendToBack(xQueueButtons, &button_data, 0);

		vTaskDelay(14 / portTICK_RATE_MS); // Set display function to run at 75Hz
	}
}

/*void vEncoder ( void *pvParameters ){
	//int prev_state = 0;
	xSemaphoreTake(xBinarySemaphoreEncoder_1, 0);

	while(1){
		//vPrintString( "E\n" );

		encoder_1 = encoder_quad();
		xSemaphoreTake(xBinarySemaphoreEncoder_1, portMAX_DELAY);

	}
}*/



void vPWM(void *pvParameters){
	unsigned long period;	// Period of PWM output.
	period = SysCtlClockGet () / PWM_DIVIDER / PWM4_RATE_HZ;


	PWM_DATA.duty = 50;
	PWM_DATA.direction = 0;

	while(1){
		PWM_DATA = speed_feedback(0, encoder_1.encoder, 0, PWM_DATA);
		PWM_direction(PWM_DATA);
		PWM_duty(PWM_DATA, period);
		vTaskDelay(33 / portTICK_RATE_MS); // Set display function to run at 30Hz
	}
}




void vFilterSpeed( void *pvParameters ){
	circBuf_t speed_buffer; // Buffer
	initCircBuf (&speed_buffer, BUF_SIZE);
	xSemaphoreTake(xBinarySemaphoreFilter, 0);
	float speed_to_store = 0;
	float buffed_speed_ = 0;

	while(1){
		xQueueReceive(xQueueSpeed, &speed_to_store, 0);
		speed_buffer = store_speed(speed_to_store, speed_buffer);
		buffed_speed_ = analysis_speed(speed_buffer);
		xQueueSendToBack(xQueueBuffedSpeed, &buffed_speed_, 0);
		xSemaphoreTake(xBinarySemaphoreFilter, portMAX_DELAY);
	}
}

void vDisplayTask( void *pvParameters ){
	initDisplay(); // intialise the OLED display
	GPS_DATA_DECODED_s GPS_DATA_DECODED;
	button_data_s button_data;
	float buffed_speed_ = 0;

	acc_time_s acc_time;

	int screen = 0;

	/* As per most tasks, this task is implemented in an infinite loop. */
	while(1) {
		/* Print out the name of this task. */
		//vPrintString( "Display Task Runing...\n" );
		GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_7);
		GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5);
		xQueueReceive(xQueueGPSDATA, &GPS_DATA_DECODED, 0);
		xQueueReceive(xQueueButtons, &button_data, 0);
		xQueueReceive(xQueueBuffedSpeed, &buffed_speed_, 0);

		screen = read_button_screen(button_data, screen,1);// GPS_DATA_DECODED.fix_s);
		set_speed = set_speed_func(set_speed, button_data, screen, GPS_DATA_DECODED.speed_s);

		display(screen, 0, 0, set_speed, GPS_DATA_DECODED, buffed_speed_, encoder_1.encoder, 0, UART_char_data_old, 0, PWM_DATA.duty, acc_time);

		vTaskDelay(66 / portTICK_RATE_MS); // Set display function to run at 15Hz
	}
}

void vFakeGPS (void *pvParameters ){
	char buf[90];
	int fake_speed = 0;
	fake_speed = 30;

	while(1){
		vPrintString( "FakeGPS runing...\n" );
		sprintf(buf, "$GPRMC,194509.000,A,4042.6142,N,07400.4168,W,%d,221.11,160412,,,A*77\n", fake_speed);
		UARTSend((unsigned char *)buf, 85, 1);
		vTaskDelay(400 / portTICK_RATE_MS); // Set display function to run at 15Hz
	}
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}


