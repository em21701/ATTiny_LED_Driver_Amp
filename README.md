ATTiny_LED_Driver
=================

LED Driver Board using and ATTiny processor for PWM and effects.

The goal of this project is to use the ATTiny as a Buck Driver for High Brightness LEDs. 
After getting a functional Buck Driver working I will add blink effects.

This driver board has 2 inputs and is intended for use as a turn signal/running light 
combo and brake light/running light combo for a motorcycle. As such the board has 2 inputs,
one for 12v power from the key and a second for 12v power from either the turn signal or 
brake light. The Key inuput will illuminate the LEDs to a low level and the second input 
will change the brightness to a significantly higher level.Blink effects will be added when 
used as a tail/brake combo to make the light more conspicuous. 

Currently planned blink effects are an invverted heartbeat and flash and fade. In the heart 
beat pattern the light will dim in a lub-dub pattern while brake light is illuminated. 
In the flash and fade pattern, the brake light will flash to full brightness then slowly fade
to a level brighter than running light level, but noticeably lower than full brightness. The
light will then flash back to full brightness and repeat.


