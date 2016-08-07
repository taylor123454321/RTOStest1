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


/* Demo includes. */
#include "demo_code\basic_io.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/* The task function. */
void vReadGPS( void *pvParameters );
void vDisplayTask( void *pvParameters );
void vFakeGPS( void *pvParameters );




/* Define the strings that will be passed in as the task parameters.  These are
defined const and off the stack to ensure they remain valid when the tasks are
executing. */
const char *pcTextForTask1 = "Task 1 is running\n";
const char *pcTextForTask2 = "Task 2 is running\n";
char UART_char_data[120];
char UART_char_data_old[120];
int index = 0;
int GPS_RUN = 0;
xSemaphoreHandle  xBinarySemaphore;
//xQueueHandle xQueueGPS;

void store_char(long UART_character, portBASE_TYPE xHigherprioritytaskWoken){
	if (UART_character == '$'){
		UART_char_data_old[0] = '\0';
		strcpy(UART_char_data_old, UART_char_data);
		index = 0;
		//read_data = 0;
		UART_char_data[index] = UART_character;
		index++;
		xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherprioritytaskWoken);
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
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_7);
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5);
}


void Timer0IntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}
void Timer1IntHandler(void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

/*-----------------------------------------------------------*/


int main( void ) {
	reset_peripheral();
	initClock();
	initPin();
	initTimer();
	initGPIO();
	initConsole();
	send_data();
	vSemaphoreCreateBinary(xBinarySemaphore);
	//xQueueGPS = xQueueCreate( 95, sizeof(UART_s ));

	/* Create tasks. */
	xTaskCreate( vReadGPS, "GPS Read Task", 240, NULL, 2, NULL );
	xTaskCreate( vDisplayTask, "Display Task", 300, NULL, 1, NULL );
	//xTaskCreate( vFakeGPS, "FakeGPS Task", 300, NULL, 1, NULL );


	IntMasterEnable();

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();	
	
	for( ;; );
}
/*-----------------------------------------------------------*/

void vReadGPS( void *pvParameters ){
	xSemaphoreTake(xBinarySemaphore, 0);
	GPS_DATA_DECODED_s GPS_DATA_DECODED;
	for( ;; ) {
		//vPrintString( "readGPS runing...\n" );

		//GPS_RAW_DATA = store_char(GPS_RAW_DATA, UART_character);
		GPS_RUN ++;
		GPS_DATA_DECODED = split_data(UART_char_data_old, GPS_DATA_DECODED);
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
	}
}

void vDisplayTask( void *pvParameters ){
	initDisplay(); // intialise the OLED display

	char stringA[30]; char stringB[30]; char stringC[30]; char stringD[30]; char stringE[30]; char stringF[30]; char stringG[30];
	int i = 0;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; ) {
		/* Print out the name of this task. */
		//vPrintString( "Display Task Runing...\n" );
		//vPrintString( UART_char_data );


		sprintf(stringA, "Screen Working");
		sprintf(stringB, "i = %d    ", i);
		sprintf(stringC, "  ");
		sprintf(stringD, "  ");
		sprintf(stringE, "GPS_RUN = %d  ",GPS_RUN);
		sprintf(stringF, "  ");
		sprintf(stringG, "  ");
		RIT128x96x4StringDraw (stringA, 6, 12, 15);
		RIT128x96x4StringDraw (stringB, 6, 24, 15);
		RIT128x96x4StringDraw (stringC, 6, 34, 15);
		RIT128x96x4StringDraw (stringD, 6, 46, 15);
		RIT128x96x4StringDraw (stringE, 6, 58, 15);
		RIT128x96x4StringDraw (stringF, 6, 70, 15);
		RIT128x96x4StringDraw (UART_char_data_old, 6, 82, 15);
		i = i + 1;

		vTaskDelay(66 / portTICK_RATE_MS); // Set display function to run at 15Hz
	}
}

void vFakeGPS (void *pvParameters ){
	char buf[90];
	int fake_speed = 0;
	fake_speed = 30;

	for(;;){
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


