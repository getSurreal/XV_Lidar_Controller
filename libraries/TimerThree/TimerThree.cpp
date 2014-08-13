/*
 *  Interrupt and PWM utilities for 16 bit Timer3 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified Oct 2009 by Dan Clemens to work with timer3 of the ATMega1280 or Arduino Mega
 *  Modified April 2012 by Paul Stoffregen
 *  Modified again, June 2014 by Paul Stoffregen
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#include "TimerThree.h"

TimerThree Timer3;              // preinstatiate

unsigned short TimerThree::pwmPeriod = 0;
unsigned char TimerThree::clockSelectBits = 0;
void (*TimerThree::isrCallback)() = NULL;

// interrupt service routine that wraps a user defined function supplied by attachInterrupt
#if defined(__AVR__)
ISR(TIMER3_OVF_vect)
{
  Timer3.isrCallback();
}

#elif defined(__arm__) && defined(CORE_TEENSY)
void ftm2_isr(void)
{
  uint32_t sc = FTM2_SC;
  if (sc & 0x80) FTM2_SC = sc & 0x7F;
  Timer3.isrCallback();
}

#endif

