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
#define F_CPU 8000000UL		//cpu frequency 8MHz

#include <avr/io.h>			//AVR chip specific defs
#include <avr/interrupt.h>  //interrupt service routines
#include <stdlib.h>			//standard c functions

#define ADC_PANIC_VALUE 225						//bail out something went horribly wrong
#define LOW 30									//ADC Low value
#define HYSTERESIS 10							//Hysterysis range
#define BLINK 125								//Bright value
#define PANIC_COUNT 50							//number of cycles before we should bail out
#define ADC_DIVIDER 3							//number of cycles PWM value is allowed
												//to settle before a reading it taken
#define TICK_DIVIDER 100							//Divide ADC time scale to something
												//that can be seen
#define PATTERN_LENGTH 100						//max number of cycles before pattern is reset

volatile uint8_t ADC_LOW_VALUE;					//define ADC Lower Limit
volatile uint8_t ADC_HIGH_VALUE;				//define ADC Upper Limit

volatile uint8_t PanicFlag=0;					//define panic flag

volatile uint8_t ADC_Div=0;						//Secondary ADC Divider
												//to minimize flicker ADC had to be slower
												//than 30kHz but to maximize efficiency and
												//component sizes PWM should be as fast as possible

volatile uint8_t PatDiv=0;						//Counter for pattern divider
volatile uint8_t Tick=0;							//Quarter Second tick counter



void setup()
{
	CCP = 0xD8;									//enable protected register change
	// CLKPSR = 0000 -> 0
	// CLKPSR = 0001 -> DIV 2
	// CLKPSR = 0010 -> DIV 4
	// CLKPSR = 0011 -> DIV 8 (default)
	CLKPSR = 0x0000;							//CPU prescaler to 0 = 8MHz

	sei();										//turn on interrupts

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

	//8MHZ CPU div 128 (MAX) = 62.5kHz
	//ADPS = 111 -> 128
	//ADPS = 110 -> 64
	//ADPS = 101 -> 32

	ADIE = 1									//Enable ADC Interrupt
	ADATE = 1									//Auto Trigger

	ADCSRA - ADEN | ADSC | ADATE | ADIF | ADIE | ADPS2 | ADPS1 | ADPS0
*/
	ADCSRA = 0b10101111;

	ADCSRA |= (1<<ADSC);						//Start Conversion


}

void SetLow()
{
	ADC_LOW_VALUE = LOW;				//normal lower limit
	ADC_HIGH_VALUE = LOW + HYSTERESIS;  //normal upper limit
}

void SetHigh()
{
	ADC_LOW_VALUE = BLINK;					//Bright lower limit
	ADC_HIGH_VALUE = BLINK + HYSTERESIS;  	//Bright upper limit
}

ISR(ADC_vect)
{
	//check the status of the blink input and change the limits
	//if necessary
	//read value of ADCL compare to low, high and panic values
	//and set new PWM value.

	if (ADC_Div > ADC_DIVIDER)					//The circuit needs time to settle
	{											//Skip ADC_DIVIDER cycles before acting

		if (PINB & (1<<PINB1))					//if the blink input is high
		{
			PatDiv++;							//Increment Pattern divider
			if (PatDiv > TICK_DIVIDER)
			{
				Tick++;
				PatDiv=0;
			}
			switch (Tick)
			{
				case 2:
				case 4:
				case 6:
					SetLow();
					break;
				case PATTERN_LENGTH:
					Tick=0;
					break;
				default:
					SetHigh();
					break;
			}
		}
		else
		{
			SetLow();
			Tick=0;								//reset tick
			PatDiv=0;							//reset patdiv
		}
		if (ADCL > ADC_PANIC_VALUE)
		{
			OCR0A -= 5;							//drop PWM a bit
			PanicFlag++;						//increment panic counter
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
			PanicFlag = 0;						//no need to panic
		}
		else if (ADCL > ADC_HIGH_VALUE)			//if current is above set point
		{
			OCR0A -= 1;							//lets go down slowly
												//rapid drops lead to flicker
		}
		else
		{
			//PWM is perfect, don't fix what isn't broken
			PanicFlag = 0;						//any reason to panic is gone
		}
		ADC_Div = 0;							//reset ADC_DIV for the next round
	}
	else										//skip reading this time
	{
		ADC_Div++;								//increment ADC_DIV
	}

}

int main ()
{
	setup();									//Setup chip;

	while(PanicFlag < PANIC_COUNT)				//if PanicFlag is greater than PANIC_COUNT
	{											//fail out and stop processor.

	}
	OCR0A = 0;									//turn off pwm before bailing out
	return 0;
}


