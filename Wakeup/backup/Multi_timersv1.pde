 /*
    Copyright (C) 2011  Andrew Richards
    Examples demonstrating use of WAKEUP library
    
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
#include "TimerOne.h"            // NB: modified version of public TimerOne library - updates submitted
#include <SdFatUtil.h>          // Used to give readout of freeRAM

volatile int flipflop = HIGH;

// Context variables passed to sleeper functions - need to be global or static
int contextInt = 12345;


struct globalStruct {
  String sleeperName;
  int interval;
} contextStruct[MAXSLEEPERS];



void setup(void) {
  Serial.begin(9600);

  Serial.print("Free RAM: ");
  Serial.println(FreeRam());
 
  wakeup.init();      // Initial setup, must be called before any other use of wakeup
   
  Serial.print("Free slots at start = ");
  Serial.println(wakeup.freeSlots(),DEC);
  
    // Example 1 - blink whilst doing something useful
  pinMode(13,OUTPUT);  
  digitalWrite(13, LOW);
  Serial.println("Start LED blinking");  
  wakeup.wakeMeAfter(callbackSimple, -100, NULL, TREAT_AS_ISR);    // Toggle LED 13 every 100mS

  // Example 2 - single shot wakeup with context
  Serial.println("Print out context in 3 secs");
  wakeup.wakeMeAfter(callbackWithInt, 3000, &contextInt, TREAT_AS_ISR);
  
  // Example 3 - single shot wakeup of larger function run in normal mode (ie, not as an extended ISR)
  Serial.println("Start larger function in 5 secs");
  wakeup.wakeMeAfter(callbackComplex, 5000, NULL, TREAT_AS_NORMAL);
  
  // Example 4 - multiple single-shot wakeups, with structure context - demonstrate test for exhausted slots
  Serial.print("Free slots left = ");
  Serial.println(wakeup.freeSlots(),DEC);
  Serial.println("Test for exhausted slots (should be 3)");

  long interval = 8000;
  for (int i = 0; i < MAXSLEEPERS; i++) {
    contextStruct[i].sleeperName = "Sleeper num = " + i;    // Not the most efficient, but demonstrates passing a structure as context
    contextStruct[i].interval = interval;
   
    if (wakeup.wakeMeAfter(callbackWithStruct, interval, &contextStruct[i], TREAT_AS_NORMAL) == false) {
      Serial.print("Max sleepers exceeded @ ");
      Serial.println(i);
    }
    
    interval += 100;
  }
  
   
  Serial.print("Setup complete after ");
  Serial.print(millis());
  Serial.println(" ms");
}


void loop() {
  delay(1000);

  wakeup.runAnyPending();                     // Place this call sufficiently frequently in main code to allow pending sleepers to run

  if (millis() < 15000) {
    Serial.print(millis()/1000 );
    Serial.print(".");
    Serial.print((millis()%1000)/100);
    Serial.print("s and counting.  ");
    Serial.print("Free slots left = ");
    Serial.println(wakeup.freeSlots(),DEC);
  }

}


void callbackSimple(void *dummy) {            // All callback functions need to include a void pointer, even if not used
  digitalWrite(13, flipflop ^= 1);
}


void callbackComplex(void *dummy) {
  Serial.println("Start of something complex you wouldn't run with interrupts disabled");
  delay(2000);
  Serial.println("Finished doing something complex.  Get temperature");
}

void callbackWithInt(void *context) {  
  int temp = *(int*)context;                // Cast the void pointer to an int
  
  Serial.print("Context = ");
  Serial.println(temp);  
}

void callbackWithStruct(void *context) {
  globalStruct structure = (*(globalStruct*)context);      // Cast the void pointer to the structure
  String name = structure.sleeperName;   
  int interval = structure.interval;
  
  Serial.print(name);
  Serial.print(" time since triggered = ");
  Serial.print(interval);
  Serial.println(" ms");
}



