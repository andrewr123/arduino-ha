An evolving project to develop an HA controller based on zoned Arduino Megas with Ethernet shields.

Vision is for each Arduino to be configured using an on-startup JSON file detailing the sensors and 
rule-set for decision-making, and then to act as a web server supporting a browser front-end access 
using dynamic HTML and Ajax technologies. Multiple sensors need to be monitored/controlled, so 
supplementary hardware will be used to handle multiple interrupts, activate relays etc, and slave 
ATMega328s used to handle asynchronous data input

Stable upload so far is WAKEUP library with examples.  Others are work in progress