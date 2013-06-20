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

#define ADC_PANIC_VALUE 250						//bail out something went horribly wrong
#define LOW 30									//ADC Low value
#define HYSTERESIS 10							//Hysterysis range
#define BLINK 150								//Bright value

volatile uint8_t ADC_LOW_VALUE;						//define ADC Lower Limit
volatile uint8_t ADC_HIGH_VALUE;					//define ADC Upper Limit

volatile uint8_t PanicFlag;						//define panic flag

void setup()
{

	PanicFlag = 0;								//set panic flag to false
	sei();										//turn on interrupts

	ADC_LOW_VALUE = LOW;						//Initialize low value
	ADC_HIGH_VALUE = LOW + HYSTERESIS;			//Initialize High Value

	//PWM SETUP
/*	TCCR0B |= (1<<CS00);
	TCCR0B &= ~((1<<CS01) | (1<<CS02));			//Internal Clock no prescaler
	TCCR0B |= (1<<WGM02);
	TCCR0B &= ~(1<< WGM03);				 		//Fast 8 bit PWM B
	TCCR0B &= ~(1<<ICNC0);						//Turn Off Noise canceling
	TCCR0B |= (1<<ICES0);						//Incoming edge select - rising
*/
	//TCCR0B - ICNC0 | ICES0 | - | WGM03 | WGM02 | CS02 | CS01 | CS00
	TCCR0B = 0b01001001;						//same as above in 1 operation

/*
	TCCR0A |= (1<<WGM00);
	TCCR0A &= ~(1<<WGM01);						//Fast 8 bit PWM A
	TCCR0A &= ~(1<<COM0B0);
	TCCR0A |= (1<<COM0B1);						//non-inverting mode
	TCCR0A &= ~(1<<COM0A0);
	TCCR0A |= (1<<COM0A1);						//non-inverting Mode
*/
	//TCCR0A = COM0A1 | COM0A0 | COM0B1 | COM0B0 | - | - | WGM01 | WGM00
	TCCR0A = 0b10100001;

	TCCR0C &= ~((1<<FOC0A) | (1<<FOC0B));		//N/A for PWM must be 0

	DDRB = 0b0001;								//pin1 out all others in
	PUEB = 0b0001;								//enable internal pullup
	PORTB = 0;									//set pins to high impedance
	OCR0A = 10;									//start PWM at a low value
	OCR0B = 0;

	//ADC SETUP
	PRR &= ~(1<<PRADC);							//turn off adc power reduction

	ADMUX &= ~(1<<MUX0);
	ADMUX |= (1<<MUX1);							//Select PB2 as ADC input

	ADCSRB &= ~((1<<ADTS2) |
				 (1<<ADTS1) |
				 (1<<ADTS0));					//ADC Trigger Source Free Running

	DIDR0 |= (1<<ADC2D);						//Disable digital input
/*
	ADCSRA |= (1<<ADEN);						//enable ADC
	ADCSRA &= ~(1<<ADPS2);						//Set prescaler to div 4
	ADCSRA |= ((1<<ADPS1) |						//125kHz @ 1MHz CPU
			 (1<<ADPS0));						//required range 50-200kHz

	ADCSRA |= (1<<ADIE);						//Enable ADC Interrupt
	ADCSRA |= (1<<ADATE);						//Auto Trigger
*/
//	ADCSRA - ADEN | ADSC | ADATE | ADIF | ADIE | ADPS2 | ADPS1 | ADPS0
	ADCSRA = 0b10101011;						//same as above in one operation

	ADCSRA |= (1<<ADSC);						//Start Conversion

}

ISR(ADC_vect)
{
	//read value of ADCL compare to low, high and panic values
	//and set new PWM value.

	//There is a race condition here but I don't think it will be an
	//issue. ISR must complete and OCR0A must updated and settle before
	//the next conversion starts to get a good reading.

	if (ADCL > ADC_PANIC_VALUE)
	{
		OCR0A = 0;					//set PWM to 0
		PanicFlag++;				//force the chip to end processing
	}
	else if (ADCL < ADC_LOW_VALUE)	//if current is below set point
	{
		OCR0A += 1;					//ramp up to the value
	}
	else if (ADCL > ADC_HIGH_VALUE)	//if current is above set point
	{
		OCR0A -= 2;					//lets ramp down a little faster than up
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
	}

	return 0;
}


