/*
 * blinky.c
 *
 *  Created on: May 11, 2013
 *      Author: Eric Morrison
 */

#include <avr/io.h>			//define port settings
#include <avr/sleep.h>		//sleep functions
#include <avr/wdt.h>		//watch dog timer functions
#include <avr/interrupt.h>  //interrupt functions
#include <util/delay.h>		//delay functions

void setup()
{
	DDRB = 0b0001;		//set Pin 1 to output others to input

}

int main ()
{
	setup();
	while (1)
	{
		PORTB ^= (1<<0);	//toggle bit 0 (XOR 1 to position 0)
		_delay_ms(50);		//delay 100ms

		//delay function is said to have a lot of baggage, good
		//for first try but sleep/wdt is better

	}
}
