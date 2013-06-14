/*
 * buck.c
 *
 *  Created on: June 2, 2013
 *      Author: Eric Morrison
 *
 *					 ATTINY 10
 *					   _____
 *      MOSFET ---- <1 |	 | 6  ---- RESET (PROGRAMMING ONLY)
 *         GND ----  2 |     | 5  ---- VCC
 *       BLINK ---- >3 |_____| 4< ---- MEASURE
 */

//#define __AVR_ATtiny5__		//chip definition
#define F_CPU 1000000UL		//cpu frequency

#include <avr/io.h>			//AVR chip specific defs
#include <avr/interrupt.h>  //interrupt service routines
#include <stdlib.h>			//standard c functions

#define ADC_LOW_VALUE	80	//current level = (I*30)/(Vin/255)
#define ADC_HIGH_VALUE	90	//high current level to give a little hysteresis
#define ADC_PANIC_VALUE 150	//bail out something went horribly wrong

int PanicFlag;

void setup()
{

	PanicFlag = 0;								//set panic flag to false
	sei();										//turn on interrupts

	//PWM SETUP
	TCCR0B = (1<<CS00) | (0<<CS01) | (0<<CS02);	//Internal Clock no prescaler
	TCCR0B |= (1<<WGM02) | (0<< WGM03); 		//Fast 8 bit PWM B
	TCCR0B |= (0<<ICNC0);						//Turn Off Noise canceling
	TCCR0B |= (1<<ICES0);						//Incoming edge select - rising

	TCCR0A = (1<<WGM00) | (0<<WGM01);			//Fast 8 bit PWM A
	TCCR0A |= (0<<FOC0A) | (0<<FOC0B);			//N/A for PWM must be 0
	TCCR0A |= (0<<COM0B0) | (0<<COM0B1);
	TCCR0A |= (0<<COM0A0) | (1<<COM0A1);		//non-Inverting Mode


	DDRB = 0b0001;								//pin1 out all others in
	PUEB = 0b0001;								//enable internal pullup
	PORTB = 0;									//set pins to high impedance
	OCR0A = 0;
	OCR0B = 0;

	//ADC SETUP
	PRR &= ~(1<<PRADC);							//turn off adc power reduction

	ADMUX &= ~(1<<MUX0);
	ADMUX |= (1<<MUX1);							//Select PB2 as ADC input

	ADCSRA &= ~((1<<ADPS2) |
			 (1<<ADPS1) |
			 (1<<ADPS0));						//Set pre scaler to div2 (lowest)
	ADCSRA |= (1<<ADIE);						//Enable ADC Interrupt
	ADCSRA &= ~(1<<ADATE);						//Single Conversion

	ADCSRB &= ~((1<<ADTS2) |
			 (1<<ADTS1) |
			 (1<<ADTS0));						//ADC Trigger Source Free Running

	DIDR0 |= (1<<ADC2D);						//Disable digital input

	ADCSRA |= (1<<ADEN);						//enable ADC
	ADCSRA |= (1<<ADSC);						//Start Conversion

}

ISR(ADC_vect)
{
	//read value of ADCL compare to low high and panic values
	//and set new PWM value. Conversion is stopped while PWM
	//is updated

	ADCSRA &= ~(0<<ADEN);			//make sure conversion has stopped

	if (ADCL > ADC_PANIC_VALUE)
	{
		OCR0A = 0;					//set PWM to 0
		PanicFlag++;				//force the chip to end processing
	}
	else if (ADCL < ADC_LOW_VALUE)	//if current is below set point
	{
		OCR0A += ADC_LOW_VALUE-ADCL;//bump it up by the difference
									//between the set and actual
	}
	else if (ADCL > ADC_HIGH_VALUE)	//if current is above set point
	{
		OCR0A -= ADCL-ADC_LOW_VALUE;//bump it down by the difference
									//between the set and actual
	}
	else
	{
		//life is good do nothing
	}

//	OCR0A = 100;						//just show me something

	ADCSRA |= (1<<ADEN);			//start new conversion
}

int main ()
{
	setup();									//Setup chip;

	while(PanicFlag < 1){};						//Infinite do nothing loop
												//real work is done by ISR
												//if PanicFlag is greater
												//than 0 fail out and stop processor.
	return 0;
}


