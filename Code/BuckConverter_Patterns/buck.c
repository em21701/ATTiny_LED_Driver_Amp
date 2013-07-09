/*
 * buck.c
 *
 *  Created on: June 2, 2013
 *      Author: Eric Morrison
 *
 *					 ATTINY 5
 *					    _____
 *      MOSFET ---- <1 |	 | 6  ---- RESET (PROGRAMMING ONLY)
 *         GND ----  2 |     | 5  ---- VCC
 *       BLINK ---- >3 |_____| 4< ---- MEASURE
 */

#define __AVR_ATtiny5__		//chip definition
#define F_CPU 1000000UL		//cpu frequency 1MHz

#include <avr/io.h>			//AVR chip specific defs
#include <avr/interrupt.h>  //interrupt service routines
#include <stdlib.h>			//standard c functions

#define ADC_PANIC_VALUE 160						//bail out something went horribly wrong
#define LOW 30									//ADC Low value
#define HYSTERESIS 10							//Hysterysis range
#define BLINK 100								//Bright value

volatile uint8_t ADC_LOW_VALUE;					//define ADC Lower Limit
volatile uint8_t ADC_HIGH_VALUE;				//define ADC Upper Limit

volatile uint8_t PanicFlag;						//define panic flag

void setup()
{

	PanicFlag = 0;								//set panic flag to false
	sei();										//turn on interrupts

	ADC_LOW_VALUE = LOW;						//Initialize low value
	ADC_HIGH_VALUE = LOW + HYSTERESIS;			//Initialize High Value

	//PWM SETUP
/*	CS00 = 1
	CS01 & CS02 = 0								//Internal Clock no prescaler
	WGM02 = 1
	WGM03 = 0							 		//Fast 8 bit PWM B
	ICNC0 = 0									//Turn Off Noise canceling
	ICES0										//Incoming edge select - rising

	TCCR0B - ICNC0 | ICES0 | - | WGM03 | WGM02 | CS02 | CS01 | CS00
*/
	TCCR0B = 0b01001001;

/*
	WGM00 = 1
	WGM01 = 0									//Fast 8 bit PWM A
	COM0B0 = 0
	COM0B1 = 1									//non-inverting mode
	COM0A0 = 0
	COM0A1 = 1									//non-inverting Mode

	TCCR0A = COM0A1 | COM0A0 | COM0B1 | COM0B0 | - | - | WGM01 | WGM00
*/
	TCCR0A = 0b10100001;

	TCCR0C &= ~((1<<FOC0A) | (1<<FOC0B));		//N/A for PWM must be 0

	DDRB = 0b0001;								//pin1 out all others in
	PUEB = 0b0001;								//enable internal pullup
	PORTB = 0;									//set pins to high impedance

	OCR0B = 0;									//not used, set to 0

	//ADC SETUP
	PRR &= ~(1<<PRADC);							//turn off adc power reduction

	ADMUX &= ~(1<<MUX0);
	ADMUX |= (1<<MUX1);							//Select PB2 as ADC input

	ADCSRB &= ~((1<<ADTS2) |
				 (1<<ADTS1) |
				 (1<<ADTS0));					//ADC Trigger Source Free Running

	DIDR0 |= (1<<ADC2D);						//Disable digital input
/*
	ADEN = 1									//enable ADC

	ADPS1 = 0									//Set prescaler to div 32
	ADPS0 & ADPS2 = 1							//31.25kHz @ 1MHz CPU
			 									//trial and error has shown the
			 	 	 	 	 	 	 	 	 	//least amount of flicker here

	ADIE = 1									//Enable ADC Interrupt
	ADATE = 1									//Auto Trigger

	ADCSRA - ADEN | ADSC | ADATE | ADIF | ADIE | ADPS2 | ADPS1 | ADPS0
*/
	ADCSRA = 0b10101101;

	ADCSRA |= (1<<ADSC);						//Start Conversion

}

ISR(ADC_vect)
{
	//check the status of the blink input and change the limits
	//if necessary
	//read value of ADCL compare to low, high and panic values
	//and set new PWM value.


	if (PINB & (1<<PINB1))					//if the blink input is high
	{
		ADC_LOW_VALUE = BLINK;				//bright lower limit
		ADC_HIGH_VALUE = BLINK + HYSTERESIS;//bright upper limit

	}
	else
	{
		ADC_LOW_VALUE = LOW;				//normal lower limit
		ADC_HIGH_VALUE = LOW + HYSTERESIS;  //normal upper limit
	}

	if (ADCL > ADC_PANIC_VALUE)
	{
		OCR0A = 0;							//set PWM to 0
		PanicFlag++;						//force the chip to end processing
	}
	else if (ADCL < ADC_LOW_VALUE)			//if current is below set point
	{
		if ((ADC_LOW_VALUE - ADCL) > 30) 	//if it is way below
		{
			OCR0A += 10;					//ramp up to the value quickly
		}
		else								//otherwise
		{
			OCR0A += 1;						//lets go slowly to avoid overshoot
		}
	}
	else if (ADCL > ADC_HIGH_VALUE)			//if current is above set point
	{
		OCR0A -= 2;							//lets go down slowly
											//rapid drops lead to flicker so
											//down slowly is the best plan

	}
	else
	{
		//life is good do nothing
	}


}

int main ()
{
	setup();									//Setup chip;

	while(PanicFlag < 1)						//if PanicFlag is greater than 0
	{											//fail out and stop processor.

	}

	return 0;
}


