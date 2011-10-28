/* Timing loop to calibrate WAKEUP library

  Sampled with logic analyser on pin2

  For all tests:  MAXSLEEPERS: 256
                  CODEOVERHEAD: 8us per sleeper
                  Pin2 toggle target = 10ms
                 
  Num sleepers:                1          8            32            64          128            256
  Actual toggle speed (ms)   10.03      10.03        10.03          10.02        10.01          9.99
  


**************************/



#include "Wakeup.h"
#include "TimerOne.h"

volatile int flipflop = HIGH;

void setup(void) {
  Serial.begin(9600);
  pinMode(2,OUTPUT);
  
  wakeup.init();
  wakeup.wakeMeAfter(sleeperPin2, -10, NULL, true);                                     // Toggle pin2 every 10ms
  delay(1000);
  for (int i = 0; i < 7; i++) wakeup.wakeMeAfter(sleeperNull, 10000, NULL, TREAT_AS_ISR);  // 8 sleepers 
  delay(1000);
  for (int i = 0; i < 24; i++) wakeup.wakeMeAfter(sleeperNull, 10000, NULL, TREAT_AS_ISR);   // 32 sleepers
  delay(1000);
  for (int i = 0; i < 32; i++) wakeup.wakeMeAfter(sleeperNull, 10000, NULL, TREAT_AS_ISR);   // 64 sleepers
  delay(1000);
  for (int i = 0; i < 64; i++) wakeup.wakeMeAfter(sleeperNull, 10000, NULL, TREAT_AS_ISR);   // 128 sleepers
  delay(1000);
  for (int i = 0; i < 128; i++) wakeup.wakeMeAfter(sleeperNull, 10000, NULL, TREAT_AS_ISR);   // 256 sleepers


}

void sleeperPin2(void *context) {
  digitalWrite(2, flipflop ^= 1);
}  

void sleeperNull(void *context) {
} 


void loop() {
}




