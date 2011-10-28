 /*
    Copyright (C) 2011  Andrew Richards
    Master library for Home Automation system

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

 
#ifndef HomeAutom_h
#define HomeAutom_h

#include "WProgram.h"

#define FIFOLEN 128								// Must be multiple of 8; max 256
//#define MAXSLEEPERS 8							// Must be multiple of 8; max 256
//#define MAXPENDING 8
//#define MAXHEARTBEAT 8250						// Round down from absolute max of 8,388,480 uS
//#define CODEOVERHEAD (25 + (MAXSLEEPERS * 2))	// Logic analyser-determined to allow for duration of WAKEUP code

class FIFO {
public:
  void init();
  void put(byte);
  byte get();
  byte get(byte);
  byte size();
private:
  byte _buffer [FIFOLEN];
  byte _putPtr;
  byte _getPtr;	
};

class BITSTRING {
public:
  void init(byte *buffer, byte bufLen);
  void putBit(byte bitNum, boolean bitVal);
  byte getBit(byte bitNum);
private:
  byte *_buffer;
};




#endif
