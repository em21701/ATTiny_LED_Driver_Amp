/*
 * fade.c
 *
 *  Created on: May 12, 2013
 *      Author: eric
 */


#include <avr/io.h>

#include <util/delay.h>		//delay functions

void setup()
{

	TCCR0B = (1<<CS00) | (0<<CS01) | (0<<CS02);	//Internal Clock no prescaler
	TCCR0B |= (1<<WGM02) | (0<< WGM03); 		//Fast 8 bit PWM B
	TCCR0B |= (0<<ICNC0);						//Turn Off Noise canceling
	TCCR0B |= (1<<ICES0);						//Incoming edge select - rising

	TCCR0A = (1<<WGM00) | (0<<WGM01);			//Fast 8 bit PWM A
	TCCR0A |= (0<<FOC0A) | (0<<FOC0B);			//N/A for PWM must be 0
	TCCR0A |= (0<<COM0B0) | (0<<COM0B1);
	TCCR0A |= (1<<COM0A0) | (1<<COM0A1);

	DDRB = 0b0001;


}

int main ()
{
	setup();

	OCR0A = 0;
	OCR0B = 0;

	while(1)
	{
		for (int a=25;a<255;a=a+5)		//fade out (sinking current)
		{
			OCR0A=a;
			//OCR0B=a;
			_delay_ms(20);
		}
		for (int a=255;a>25;a=a-5)		//fade in (sinking current)
		{
			OCR0A=a;
			_delay_ms(20);
		}
	}


}
