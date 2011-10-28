 /*  
	*****************  WAKEUP  **********************
	
	Description
	-----------
	
	Permits multiple 'sleeper' functions to be woken after a specified interval, allowing the main program
	to get on with other things in the meantime (in contrast to delay() or repeated polling).  This enables
	a crude form of co-operative processing or 'protothreading'

	Sleepers are woken after a defined interval in mS with a pointer to a context established when put to sleep
	This context needs to be interpreted appropriately by the sleeper, and must be global or static
	
	Sleepers can be woken in two modes:
		  - as an extended Interrupt Service Routine woken exactly after the defined delay - in which case 
			they run with interrupts DISABLED and should therefore avoid lengthy processing
		  - as an ordinary function available to run after the defined delay.  The function is run in 
			response to a poll by the main program calling runAnyPending()
	
	Comments use a lighthearted analogy of 'sleepers' in 'bunks'.  New sleepers are put in the first free bunk
	starting from the bottom (index 0).  When a sleeper wakes and leaves, the topmost sleeper is moved to the 
	newly-vacant bunk, thus optimising access to free bunks above
	
	Functions available
    -------------------
    
    - init                 Must be called before first use of WAKEUP class
    - wakeMeAfter          Request a nominated function to be called in the future. Returns false if not slots free.  Arguments:
        - sleeper          The function to be called
        - ms               The delay, in mS
        - context          A pointer to data providing the called function with the context for the call
        - treatAsISR       A flag indicating whether the function is to be called as an extended Interrupt Service Routine
                           (fast, but limited processing allowed) or as a normal function in response to a poll by the main
                           programme (speed of response depends on polling frequency, but much more can be done safely
    - runAnyPending        Called by the main program (frequently) to allow normal sleepers to run (once they've woken)
    - freeSlots            Returns the number of sleeper slots left    
    
	
	Version history
	---------------
	
	Version 1.0 Oct 2011 - Initial release, Andrew Richards
	
	Licensing
	---------

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



#include "WProgram.h"
#include "Wakeup.h"
#include "TimerOne.h"


void WAKEUP::init() {
  _numSleepers = 0;			// No sleepers
  _numPending = 0;
  _inISR = false;
  Timer1.initialize();
}



boolean WAKEUP::wakeMeAfter( void (*sleeper)(void*), long ms, void *context, boolean treatAsISR) {
  unsigned long timeSinceLast;
   
  // Check the time requested and if there is a free bunk to store the sleeper
  if (ms == 0 || _numSleepers >= MAXSLEEPERS) return false;
 
  _oldSREG = SREG;
  cli();
  
  // If any already sleeping, then get the count from start of heartbeat and deduct from all active timers
  if (_numSleepers > 0 && (timeSinceLast = getElapsed()) > 0) {
    for (int i = 0; i < _numSleepers; i++) _bunks[i].timeToWake -= timeSinceLast; 
  }

  // Put new sleeper into top bunk and set alarm clock
  _bunks[_numSleepers].sleepDuration = ms;					// Save the delay to inform timerISR
  _bunks[_numSleepers].callback = sleeper;					// Put sleeper into bunk
  _bunks[_numSleepers].treatAsISR = treatAsISR;				// Interrupt on wake or put on pending queue
  _bunks[_numSleepers].context = context;					// Save its context
  _bunks[_numSleepers].timeToWake = (ms > 0) ? ms : -ms;	// Set the delay 

  _numSleepers++; 
  
  SREG = _oldSREG;  
      
  startHeartbeat();    // Set counter going with an appropriate heartbeat
     
  return true;  
}


void WAKEUP::startHeartbeat() {      // Set heartbeat to longest required to wake lightest sleeper
  _heartbeat = MAXHEARTBEAT;
  
  _oldSREG = SREG;
  cli();
  for (int i = 0; i < _numSleepers; i++) {
	  if (_bunks[i].timeToWake > 0 && _bunks[i].timeToWake < _heartbeat) _heartbeat = _bunks[i].timeToWake;
  }
  SREG = _oldSREG;

  Timer1.start();
  Timer1.attachInterrupt(timerISRWrapper, _heartbeat * 1000 - (_numSleepers * CODEOVERHEAD));
}


void WAKEUP::stopHeartbeat() {
  Timer1.stop();
}

unsigned long WAKEUP::getElapsed() {
  return Timer1.read() / 1000;
}

unsigned int WAKEUP::freeSlots() {
  return MAXSLEEPERS - _numSleepers;
}

void WAKEUP::runAnyPending() {
	void (*callback)(void*);
	void *context;
	
	// Provided this isn't being called from sleeper running under timerISR, then process all pending sleepers
	if (_inISR == false) while (_numPending > 0) {			// Queue could grow dynamically as new sleepers awake	
		_oldSREG = SREG;					
		cli();							
		_numPending--;					// timerISR can change, so disable interrupts & take snapshot here
		callback = _pending[_numPending].callback;	
		context = _pending[_numPending].context;
		SREG = _oldSREG;
		
		callback(context);				// Call sleeper as normal function call - take as long as you like
	}
}

// **************  Interrupt Service Routine  *************

void WAKEUP::timerISR() {							// Runs every heartbeat 
  unsigned int runNowPtr = MAXPENDING;		                // Pointer to top end of _pending to use as temporary scratchpad
  
  for (int i = 0; i < _numSleepers; i++) {
	  
	// If countdown has finished then put on pending queue and shuffle bunks (if needed)
	if ((_bunks[i].timeToWake -= _heartbeat) <= 0) {  

		  // Put in pending queue, either to wake in a few moments or from main program using runAnyPending
		  // If runAnyPending not run fast enough then bottom-up and top-down queues may clash, so check
		  if (_bunks[i].treatAsISR == TREAT_AS_ISR) {				// Wake later in this function
  			  if (--runNowPtr <= _numPending) runNowPtr++;
			  _pending[runNowPtr].callback = _bunks[i].callback;
			  _pending[runNowPtr].context = _bunks[i].context;
		  }
		  else {													// Wake in response to runAnyPending
			  _pending[_numPending].callback = _bunks[i].callback;
			  _pending[_numPending].context = _bunks[i].context;
			  if (++_numPending >= runNowPtr) _numPending--;			
		  }
		  
		  // If others left after one-shot sleeper removed, take sleeper in topmost bunk and move to the newly-vacant bunk
	      if (_bunks[i].sleepDuration > 0) {					// Positive number means one-shot
		      _numSleepers--;			
	      	  if (_numSleepers > 0) {				// Move topmost sleeper into this bunk (v rarely might be the same one, but safe so ignore)
			      _bunks[i].callback = _bunks[_numSleepers].callback;
			      _bunks[i].treatAsISR = _bunks[_numSleepers].treatAsISR;
			      _bunks[i].sleepDuration = _bunks[_numSleepers].sleepDuration;
			      _bunks[i].timeToWake = _bunks[_numSleepers].timeToWake;
			      _bunks[i].context = _bunks[_numSleepers].context;
			      i--;								// Decrement counter to ensure this sleeper isn't missed
		      }
	      }
	      else _bunks[i].timeToWake -=_bunks[i].sleepDuration;			// Repeating sleeper, just reset countdown
        
    }
  }
  
  // Start the heartbeat if sleepers left
  if (_numSleepers == 0) stopHeartbeat(); startHeartbeat();
  
  // Run the TREAT_AS_ISR sleepers that were woken - be quick (and block runAnyPending() from being run)
  _inISR = true;
  for (int i = runNowPtr; i < MAXPENDING; i++) _pending[i].callback(_pending[i].context);
  _inISR = false;
  
}


WAKEUP wakeup;

void timerISRWrapper() {  // http://www.parashift.com/c++-faq-lite/pointers-to-members.html#faq-33.2
  wakeup.timerISR();
}

