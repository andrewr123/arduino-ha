 /*
    Copyright (C) 2011  Andrew Richards
    Examples demonstrating use of WAKEUP library
    
    Multiple variants on timing functions
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
 */



#include "Wakeup.h"
#include "TimerOne.h"           // NB: modified version of public TimerOne library - updates submitted
#include <SdFatUtil.h>          // Used to give readout of freeRAM

// *********** Counter to demonstrate work going on **************

volatile unsigned long counter;
volatile boolean keepCounting;

// ********** flag to indicate state of LED **********

volatile int flipflop = HIGH;

// ***********  Context variables passed to sleeper functions - need to be global or static  ***********
int contextInt = 12345;

struct globalStruct {
  String reply;
  int num;
  int interval;
} contextStruct[MAXSLEEPERS];



void setup(void) {
  Serial.begin(9600);

  Serial.print("Free RAM: ");
  Serial.println(FreeRam());
 
  wakeup.init();      // Initial setup, must be called before any other use of wakeup
  counter = 0;
  keepCounting = true;
   
  Serial.print("Free slots at start = ");
  Serial.println(wakeup.freeSlots(),DEC);
  
  // Example 1 - blink whilst doing something useful
  pinMode(13,OUTPUT);  
  digitalWrite(13, LOW);
  Serial.println("Start LED blinking");  
  wakeup.wakeMeAfter(callbackSimple, -100, NULL, TREAT_AS_ISR);    // Toggle LED 13 every 100mS
  
  // Example 2 - set timer to wake up main loop every 1000ms
  wakeup.wakeMeAfter(stopCounter, -1000, (void*)&keepCounting, TREAT_AS_ISR);    // keepCounting is volatile, so need to cast 

  // Example 3 - single shot wakeup with context
  Serial.println("Print out context in 3.5 secs");
  wakeup.wakeMeAfter(callbackWithInt, 3500, &contextInt, TREAT_AS_ISR);
  
  // Example 4 - single shot wakeup of larger function run in normal mode (ie, not as an extended ISR)
  Serial.println("Start larger function in 5 secs");
  wakeup.wakeMeAfter(callbackComplex, 5000, NULL, TREAT_AS_NORMAL);
  
  // Example 5 - multiple single-shot wakeups, with structure context - demonstrate test for exhausted slots
  Serial.print("Free slots left = ");
  Serial.println(wakeup.freeSlots(),DEC);
  Serial.println("Test for exhausted slots");

  long interval = 8000;

  for (int i = 0; i < MAXSLEEPERS; i++) {  
    contextStruct[i].reply = "Sleeper num = ";      // Not very elegant, but demonstrates passing structures
    contextStruct[i].num = i;
    contextStruct[i].interval = interval;
   
    if (wakeup.wakeMeAfter(callbackWithStruct, interval, &contextStruct[i], TREAT_AS_NORMAL) == false) {
      Serial.print("Max sleepers exceeded @ ");
      Serial.println(i);
    }
    
    interval += 250;
  } 
   
  Serial.print("Setup complete after ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println();
}


void loop() {
  while(keepCounting) counter++;              // Simulate doing something useful; interrupted every 1000ms

  wakeup.runAnyPending();                     // Place this call sufficiently frequently in main code to allow pending sleepers to run

  Serial.print("@");
  Serial.print(millis()/1000 );
  Serial.print(".");
  Serial.print((millis()%1000)/100);
  Serial.print("s, keeping busy, counter =   ");
  Serial.println(counter);
  
  keepCounting = true;
}


void callbackSimple(void *dummy) {            // All callback functions need to include a void pointer, even if not used
  digitalWrite(13, flipflop ^= 1);
}

void stopCounter(void *keepCounting) {                          // Interrupt main loop every 1000ms to trigger runAnyPending()
  volatile boolean *temp = (volatile boolean*)keepCounting;     // Cast the void pointer to its parent type

  *temp = false;                                                // Causes main loop to stop
}

void callbackWithInt(void *context) {      
  int temp = *(int*)context;                // Cast the void pointer to its parent type
  
  Serial.print("Context = ");               // Wouldn't normally run blocking code (such as Serial) inside an ISR
  Serial.println(temp);  
}


void callbackComplex(void *dummy) {         
  Serial.println("Start of something complex you wouldn't run with interrupts disabled");
  delay(2000);
  Serial.println("Finished doing something complex");
}

void callbackWithStruct(void *context) {
  globalStruct structure = (*(globalStruct*)context);      // Cast the void pointer to the structure
  String reply = structure.reply;   
  int num = structure.num;
  int interval = structure.interval;
  
  Serial.print(reply);
  Serial.print(num);
  Serial.print(" time since triggered = ");
  Serial.print(interval/1000 );
  Serial.print(".");
  Serial.print((interval%1000)/100);
  Serial.print("s");
  Serial.print("  Free slots left = ");
  Serial.println(wakeup.freeSlots(),DEC);
}



