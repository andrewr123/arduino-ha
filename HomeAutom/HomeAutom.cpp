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

#include "WProgram.h"
#include "HomeAutom.h"


//***************** FIFO ************************

void FIFO::init() {
  _putPtr = 0;
  _getPtr = 0;
  for (int i = 0; i < FIFOLEN/8; i++) _buffer[i] = 0; 
}

void FIFO::put(byte val) {
  _buffer [_putPtr / 8] &= ~(1 << (_putPtr % 8) );			// Clear bit
  _buffer [_putPtr / 8] |= ((val & 1) << (_putPtr % 8) );	// Store LSB bit
  if (++_putPtr >= FIFOLEN) _putPtr--;
}

byte FIFO::get() {													// Repeated calls return next bit in buffer
  byte result = _buffer [_getPtr / 8] & (1 << (_getPtr % 8));		// Get bit from relevant byte
  result = result >> (_getPtr % 8);									// Shift bit to LSB
  if (++_getPtr >= FIFOLEN) {										// Increment counter, unless at the end
	  _getPtr--;
	  result = 2;
  }  
  return result;
}

byte FIFO::get(byte idx) {
  byte result = _buffer [idx / 8] & (1 << (idx % 8));		// Get bit from relevant byte
  result = result >> (idx % 8);									// Shift bit to LSB
  
  return result;
}

byte FIFO::size() {
  return _putPtr;
}

// *************  BITSTRING  *******************
// Provides methods for setting, clearing and testing a bit in an arbitrary length buffer (max 256 bits)
//

void BITSTRING::init(byte *buffer, byte bufLen) {			// Pass pointer to array and the length of that array (in bits)
  for (int i = 0; i < bufLen/8; i++) *(buffer+i) = 0;
  _buffer = buffer;
}

void BITSTRING::putBit(byte bitNum, boolean bitVal) {
  if (bitVal) {
	*(_buffer + (bitNum / 8)) |= _BV(bitNum % 8);		// Store LSB bit
  }
  else {
  	*(_buffer + (bitNum / 8)) &= ~_BV(bitNum % 8);		// Clear bit
  }
}

byte BITSTRING::getBit(byte bitNum) {
  byte result = *(_buffer + (bitNum / 8)) & _BV(bitNum % 8);		// Get bit from relevant byte
  result = result >> (bitNum % 8);								// Shift bit to LSB
  
  return result;
}
