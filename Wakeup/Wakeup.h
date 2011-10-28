 /*  
	*****************  WAKEUP  **********************
	
	Description
	-----------
	
	Definition file to accompany wakeup.cpp - see full description there
	
	Version history
	---------------
	
	Version 1.0 Oct 2011 - Initial release, Andrew Richards
	
	Licencing
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

 
#ifndef Wakeup_h
#define Wakeup_h

#include "WProgram.h"

#define MAXSLEEPERS 32							// Max 64k, but unworkable (out of memory and would take too long). Tested with 256; 16-32 generally enough
#define MAXPENDING 32							// Needs to be large enough to hold max number of sleepers woken in one go, assuming prompt clearance by runAnyPending()
#define MAXHEARTBEAT 8350						// in ms.  Round down from absolute max of 8,388,480 us
#define CODEOVERHEAD 8 							// Logic analyser-determined to allow for duration of WAKEUP code; calibrated @ 10mS with 8 sleepers

const boolean TREAT_AS_ISR = true;
const boolean TREAT_AS_NORMAL = false;

class WAKEUP {
public:
  void init();									// Must be called at startup
  boolean wakeMeAfter( void (*sleeper)(void*), long ms, void *context, boolean treatAsISR);	// Function to wake after expiry of ms.  If ms is negative then cycle repeats, else one-shot	
  void runAnyPending();							// Called by the main program to run any pending sleepers
  unsigned int freeSlots();						// Returns number of bunks available
  void timerISR();								// Called every _heartbeat.  Must be public to allow call by timerISRWrapper()
 
private:
  // Methods
  void startHeartbeat();						// Starts timer based on shortest time to wake
  void stopHeartbeat();							// Stops timer (when no sleepers)
  unsigned long getElapsed();					// Duration in mS since last heartbeat

  // Properties - many can be changed via an ISR, so need to be volatile
  byte _oldSREG;								// Temporary store of status register
  boolean _inISR;								// Blocks use of runAnyPending by sleepers running under ISR
  volatile unsigned long _heartbeat;			// mS frequency of checking timeToWake 
  
  volatile unsigned int _numSleepers;			// Number of sleepers - if zero then turn off heartbeat
  struct _bunk {								// 'Bunk' holding sleeper or empty
	void (*callback)(void*);					// Callback function ('sleeper')
	boolean treatAsISR;							// True if callback to be immediate (as ISR).  False if to be put on pending queue
  	long sleepDuration;							// Requested delay in mS; if negative then repeating
  	long timeToWake;							// Loaded with _sleepDuration and then decremented by time since last heartbeat
  	void *context;								// For sleeper to interpret as appropriate when woken
  }  volatile _bunks[MAXSLEEPERS];				// Dynamically reshuffled for performance - first _numSleepers slots are always valid

  volatile unsigned int _numPending;			// Updated in ISR and non-ISR code     
  struct _pend {								// A sleeper that has been woken and is ready to go
	void (*callback)(void*);					
	void *context;
  } volatile _pending[MAXPENDING];				// Top end also used as temporary scratchpad by timerISR

  	
};

extern WAKEUP wakeup;

extern void timerISRWrapper();

#endif

