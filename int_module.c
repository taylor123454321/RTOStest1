/*
 * int_module.c
 *
 *  Created on: Aug 13, 2016
 *      Author: Ryan Taylor
 */

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
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "init.h"
#include "include/FreeRTOS.h"
#include "driverlib/timer.h"
#include "include/semphr.h"


xSemaphoreHandle  xBinarySemaphoreEncoder_1;
xQueueHandle xEncoder_raw_DATA;



void EncoderINT (void){
	encoder_raw_DATA_s encoder_raw_DATA;

	portBASE_TYPE xHigherprioritytaskWoken = pdFALSE;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_7);
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5);

	encoder_raw_DATA.ul_A_Val = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7); // Read the pin
	encoder_raw_DATA.ul_B_Val = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);

	xQueueSendToBackFromISR(xEncoder_raw_DATA, &encoder_raw_DATA, 0);

	xSemaphoreGiveFromISR(xBinarySemaphoreEncoder_1, &xHigherprioritytaskWoken);
    portEND_SWITCHING_ISR(xHigherprioritytaskWoken);
}

void Timer0IntHandler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);}
void Timer1IntHandler(void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);}
