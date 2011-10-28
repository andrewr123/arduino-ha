/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified June 2011 by Lex Talionis to add a function to read the timer
 *  Modified Oct 2011 by Andrew Richards to avoid certain problems:
 *  - Add (long) assignments and casts to TimerOne::read() to ensure calculations involving tmp, ICR1 and TCNT1 aren't truncated
 *  - Ensure 16 bit registers accesses are atomic - run with interrupts disabled when accessing
 *  - Remove global enable of interrupts (sei())- could be running within an interrupt routine)
 *  - start() amended to reset prescaler and start counter at 1.  Datasheet vague on this, but experiment 
 *    shows that overflow interrupt flag gets set whilst TCNT1 == 0, resulting in a phantom interrupt.  
 *    Could disable interrupts, set TCTN1 = 0 and then renable, but this inteferes with pwm()
 *	  Error only material at very short durations (1us == 16-1 clockticks, so 0.937us)
 *  - Amended DDR assignment (in pwm()) to reflect Mega
 *  - Amended read() to replace switch statement with array lookup
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 *  See Google Code project http://code.google.com/p/arduino-timerone/ for latest
 */

#ifndef TIMERONE_cpp
#define TIMERONE_cpp


#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer1.isrCallback();
}


void TimerOne::initialize(long microseconds)
{
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  setPeriod(microseconds);
}


void TimerOne::setPeriod(long microseconds)             // AR modified for atomic access
{
  
  long cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  
  oldSREG = SREG;                               
  cli();                                                        // Disable interrupts for 16 bit register access
  ICR1 = pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
  SREG = oldSREG;
  
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock
}

void TimerOne::setPwmDuty(char pin, int duty)
{
  unsigned long dutyCycle = pwmPeriod;
  
  dutyCycle *= duty;
  dutyCycle >>= 10;
  
  oldSREG = SREG;
  cli();
  if(pin == 1 || pin == 9)       OCR1A = dutyCycle;
  else if(pin == 2 || pin == 10) OCR1B = dutyCycle;
  SREG = oldSREG;
}

void TimerOne::pwm(char pin, int duty, long microseconds)  // expects duty cycle to be 10 bit (1024)
{
  if(microseconds > 0) setPeriod(microseconds);
  if(pin == 1 || pin == 9) {
//    DDRB |= _BV(PORTB1);                                   // sets data direction register for pwm output pin
    DDRB |= _BV(PORTB5);								// AR for Mega.  Rough n' ready - needs a conditional compile
    TCCR1A |= _BV(COM1A1);                                 // activates the output pin
  }
  else if(pin == 2 || pin == 10) {
    DDRB |= _BV(PORTB2);
    TCCR1A |= _BV(COM1B1);
  }
  setPwmDuty(pin, duty);
  resume();                     // Lex - make sure the clock is running.  We don't want to restart the count, in case we are starting the second WGM
                                        // and the first one is in the middle of a cycle
}

void TimerOne::disablePwm(char pin)
{
  if(pin == 1 || pin == 9)       TCCR1A &= ~_BV(COM1A1);   // clear the bit that enables pwm on PB1
  else if(pin == 2 || pin == 10) TCCR1A &= ~_BV(COM1B1);   // clear the bit that enables pwm on PB2
}


void TimerOne::attachInterrupt(void (*isr)(), long microseconds)
{
  if(microseconds > 0) setPeriod(microseconds);
  isrCallback = isr;                                       // register the user's callback with the real ISR
  TIMSK1 = _BV(TOIE1);  // sets the timer overflow interrupt enable bit
  
  resume();
}


void TimerOne::detachInterrupt()
{
  TIMSK1 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit 
}

void TimerOne::resume()                         // AR suggested
{ 
  TCCR1B |= clockSelectBits;
}

void TimerOne::restart()                // Depricated - Public interface to start at zero - Lex 10/9/2011
{
        start();                                
}

void TimerOne::start()  				// AR modified
{
  unsigned int tcnt1;
  
  GTCCR |= _BV(PSRSYNC);                // AR added - reset prescaler (NB: shared with all 16 bit timers);

  oldSREG = SREG;                       // AR - save status register
  cli();                                // AR - Disable interrupts
  TCNT1 = 1;                    		// AR set to 1 (rather than 0) to avoid phantom interrupt
  SREG = oldSREG;                       // AR - Restore status register
  
  TCCR1B |= clockSelectBits;
}


void TimerOne::stop()
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
}


unsigned long TimerOne::read()          //returns the value of the timer in microseconds
{                                       //rember! phase and freq correct mode counts up to then down again
        unsigned long tmp;                              	// AR amended to hold more than 65536 (could be nearly double this)
        unsigned int tcnt1;                             	// AR added
        const char scaleLookup[] = { 0, 0, 3, 6, 8, 10 };	// AR added to replace switch stmt - { ignore, full xtal, x8, x64, x256, x1024 }

        char scale = scaleLookup[clockSelectBits];
        
        oldSREG= SREG;
        cli();                                                  
        tmp=TCNT1;                                      
        SREG = oldSREG;
        
        do {    // Nothing -- max delay here is ~1023 cycles.  AR modified
                oldSREG = SREG;
                cli();
                tcnt1 = TCNT1;
                SREG = oldSREG;
        } while (tcnt1==tmp); //if the timer has not ticked yet

        //if we are counting down add the top value to how far we have counted down
        tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );                // AR amended to add casts and reuse previous TCNT1
        return ((tmp*1000L)/(F_CPU /1000L))<<scale;
}
 


#endif
