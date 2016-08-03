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


/* User made library includes */
#include "init.h"

/* Demo includes. */
#include "demo_code\basic_io.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/* The task function. */
void vReadGPS( void *pvParameters );
void vDisplayTask( void *pvParameters );


/* Define the strings that will be passed in as the task parameters.  These are
defined const and off the stack to ensure they remain valid when the tasks are
executing. */
const char *pcTextForTask1 = "Task 1 is running\n";
const char *pcTextForTask2 = "Task 2 is running\n";
xSemaphoreHandle  xBinarySemaphore;

void store_char(long UART_character){
	if (UART_character == '$'){
		UART_char_data_old[0] = '\0';
		strcpy(UART_char_data_old, UART_char_data);
		index = 0;
		read_data = 0;
		UART_char_data[index] = UART_character;
		index++;
	}
	else{
		UART_char_data[index] = UART_character;
		index++;
	}
}

void UARTIntHandler(void) {
	portBASE_TYPE xHigherprioritytaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherprioritytaskWoken);


    unsigned long ulStatus;
    long UART_character = 0;
    // Get the interrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE)) {
        // Read the next character from the UART and write it back to the UART.
        //
    	//UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
        UART_character = UARTCharGetNonBlocking(UART0_BASE);
        StoreQuew(UART_character);
    }
    portEND_SWITCHING_ISR(xHigherprioritytaskWoken);
}

/*-----------------------------------------------------------*/

int main( void ) {
	reset_peripheral();
	/* Set the clocking to run from the PLL at 50 MHz.  Assumes 8MHz XTAL,
	whereas some older eval boards used 6MHz. */
	SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );


	initPin();
	vSemaphoreCreateBinary(xBinarySemaphore);

	/* Create one of the two tasks. */
	xTaskCreate( vReadGPS, "GPS Read Task", 240, NULL, 2, NULL );

	/* Create the other task in exactly the same way.  Note this time that we
	are creating the SAME task, but passing in a different parameter.  We are
	creating two instances of a single task implementation. */
	xTaskCreate( vDisplayTask, "Display Task", 300, NULL, 1, NULL );

	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();	
	
	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vReadGPS( void *pvParameters ){

	xSemaphoreTake(xBinarySemaphore, 0)
	for( ;; ) {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

		readQue();
	}
}

void vDisplayTask( void *pvParameters ){
	//void initDisplay (void);
	volatile unsigned long ul;

	// intialise the OLED display
	RIT128x96x4Init(1000000);

	char stringA[30];
	char stringB[30];
	char stringC[30];
	char stringD[30];
	char stringE[30];
	char stringF[30];
	char stringG[30];

	int i = 0;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; ) {
		/* Print out the name of this task. */
		vPrintString( "hi\n" );
		sprintf(stringA, "Screen Working");
		sprintf(stringB, "i = %d    ", i);
		sprintf(stringC, "  ");
		sprintf(stringD, "  ");
		sprintf(stringE, "  ");
		sprintf(stringF, "  ");
		sprintf(stringG, "  ");
		RIT128x96x4StringDraw (stringA, 6, 12, 15);
		RIT128x96x4StringDraw (stringB, 6, 24, 15);
		RIT128x96x4StringDraw (stringC, 6, 34, 15);
		RIT128x96x4StringDraw (stringD, 6, 46, 15);
		RIT128x96x4StringDraw (stringE, 6, 58, 15);
		RIT128x96x4StringDraw (stringF, 6, 70, 15);
		RIT128x96x4StringDraw (stringG, 6, 82, 15);

		i = i + 1;

		vTaskDelay(66 / portTICK_RATE_MS); // Set display function to run at 15Hz
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


