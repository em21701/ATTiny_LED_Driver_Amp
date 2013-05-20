/*
 * fade.c
 *
 *  Created on: May 12, 2013
 *      Author: eric
 */


#include <avr/io.h>

#include <util/delay.h>		//delay functions

#define DELAY	80			//delay length in ms
#define LOW 	80			//low PWM level
#define HIGH	255			//high PWM level

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

	DDRB = 0b0001;								//pin1 out all others in
	PORTB=0;									//set pins to high impedance
	OCR0A = 0;
	OCR0B = 0;

}

int main ()
{
	setup();

	int a=0; 		//counter variable

	while(1)
	{
		while (PINB & (1<<PINB1))
		{
			if (a==10 || a==15)
			{
				OCR0A = LOW;
			}
			else
			{
				OCR0A = HIGH;
			}
			_delay_ms(DELAY);
			a++;
			if (a>=16)
			{
				a=0;
			}
		}
		a=0;
		OCR0A=LOW;
	}


}
