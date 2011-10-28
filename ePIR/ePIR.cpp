/*
 *  ePIR.cpp created for ePIR library project on 09/01/2010 18:00:00.
 *  Written using AVR Project IDE v1.97.
*/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *      
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *      
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301, USA.
*/

/*
 * * * * * * * * * * * * * * * * * * * *
 * *  Code written by Corey Johnson  * *
 * *  September 1, 2010              * *
 * * * * * * * * * * * * * * * * * * * *
*/

/*
 *  This is a library for the 'Zilog ePIR Motion Detection -Zdots SBC(Single Board Computer)'. This library was 
 *  written for the Arduino Mega. I'm sure with a little work it can be ported to other boards. However, I have
 *  no current plans to do so. The example program shows some various uses of the library and is well commented.
 *  For hardware configuration see the .txt file that came with the library or go to the Zilog website for docu-
 *  mentation about the hardware. This is the first working library I've ever written so if you have any
 *  comments or suggestions send an email to trlrtrsh2@cox.net. All I ask is that if you make any improvements
 *  to this library please give credit to the original author(Me), give the library a new version/revision number,
 *  and please send me a copy so that I may learn from it. I am not an expert at programming, but I'm always eager
 *  to learn new things. I hope you find this library useful. Enjoy!  ;o)
 
 *  Amended Andrew Richards Sep 2011:
 *  - reference Serial1, Serial2 and Serial3 by pointer 
 *  - add simplified init() function in situations where MDR and SLP pins are not required
*/
 
#include "ePIR.h"

/////////////////////////// * Constants * /////////////////////////////
const char ACK = char(6); // ..... "Acknowledge"
const char NACK = char(21); // ... "Non-Acknowledge"

///////////////////////// * Global variables * ////////////////////////
static byte MDR_Pin; // ....... Motion Detect/Reset. (Arduino pin) 
static byte SLP_Pin; // ....... Sleep. (Arduino pin)
static HardwareSerial *serialPtr;		// AR replacement of static byte Serial_Port.  Holds pointer to Serial instance

ePIR::ePIR(){/* nothing to construct */}
ePIR::~ePIR(){/* nothing to destruct */}

/* *********************** PRIVATE FUNCTIONS *********************** */
////////////////////////// * Read Function * //////////////////////////
char ePIR::readChar(char command){ // ......... Returns value selected by command sent to ePIR.
	char outChar = '\0'; // ..................... Assign NULL to output variable.

	do {	
		(*serialPtr).print(command);
		while ((*serialPtr).available() == 0);
		outChar = (*serialPtr).read();
	} while (outChar == NACK);
	
	return outChar; // .......................... Return from function with value from ePIR.
} // .......................................... End of read function.

//////////////////////// /* Write Function */ /////////////////////////
char ePIR::writeChar(char command, char inChar){ // ... Changes value of ePIR selected by command to value of inChar and returns with ACK. 
	char outChar = '\0'; // ............................. Assign NULL to output variable.

	do {
		do {	
			(*serialPtr).print(command);
			while ((*serialPtr).available() == 0);
		} while ((*serialPtr).read() == NACK);
		(*serialPtr).print(inChar);
		while ((*serialPtr).available() == 0);
		outChar = (*serialPtr).read();
	} while (outChar != ACK);
	
	return outChar; // .................................. Return from function with value 'ACK'.
} // .................................................. End of Write function.

///////////////////// /* Confirmation Function */ /////////////////////
char ePIR::confirm(void){ // ................ Sends confirmation sequence '1234' to ePIR (Needed for .Sleep and .Reset functions).
	char outChar = '\0'; // ................... Assign NULL to output variable.
	
	for (char num = '1'; num < '5'; num++) (*serialPtr).print(num);
	while ((*serialPtr).available() == 0);
	outChar = (*serialPtr).read();
	
	return outChar; // ........................ Return from function with value 'ACK'.
}

/* *********************** PUBLIC  FUNCTIONS *********************** */
/////////////////////////// Initialize ePIR ///////////////////////////
void ePIR::Init(byte serialPort, byte MDRpin, byte SLPpin){
	MDR_Pin = MDRpin;
	SLP_Pin = SLPpin;
	digitalWrite(MDR_Pin, HIGH);
	pinMode(MDR_Pin, OUTPUT);
	digitalWrite(SLP_Pin, HIGH);
	pinMode(SLP_Pin, OUTPUT);
	switch(serialPort){
		case 2:
			serialPtr = &Serial2;
		break;
		case 3:
			serialPtr = &Serial3;
		break;
		default:
			serialPtr = &Serial1;
		break;
	}
	(*serialPtr).begin(9600);
	return;
}

// Added AR - slimmed down; no need to have separate MDR or SLP ports
void ePIR::Init(byte serialPort){
	
	switch(serialPort){
		case 2:
			serialPtr = &Serial2;
		break;
		case 3:
			serialPtr = &Serial3;
		break;
		default:
			serialPtr = &Serial1;
		break;
	}
	(*serialPtr).begin(9600);
	return;
}

///////////////////////// Motion Detect Status ////////////////////////
char ePIR::Status(void){
	char status = readChar('a');
	return status;
}

////////////////////////// Light Gate Level ///////////////////////////
byte ePIR::LightLevel(void){
	byte level = byte(readChar('b'));
	return level;
}

//////////////////////// Light Gate Threshold /////////////////////////
byte ePIR::GateThresh(word threshold){
	if (257 > threshold){
		if (256 > threshold){
			writeChar('L', char(lowByte(threshold)));
		}
		else {
			byte defValue = 100;
			writeChar('L', char(defValue));
		}
	}
	byte thresholdOut = byte(readChar('l'));
	return thresholdOut;
}

//////////////////////////// MD/R Pin Mode ////////////////////////////
char ePIR::MDRmode(char mdrMode){
	switch (mdrMode){
		case 'M':
			pinMode(MDR_Pin, INPUT);
			digitalWrite(MDR_Pin, HIGH);
			writeChar('C', 'M');
			break;
		case 'R':
			pinMode(MDR_Pin, OUTPUT);
			digitalWrite(MDR_Pin, HIGH);
			writeChar('C', 'R');
			break;
		case 'I':				// Added AR, for Interrupt-driven
			writeChar('C', 'M');
			break;
	}
	char mdrModeOut = readChar('c');
	return mdrModeOut;
}

///////////////////////// MD-Pin Active Time //////////////////////////
byte ePIR::MDtime(word mdTime){
	if (257 > mdTime){
		if (256 > mdTime){
			writeChar('D', char(lowByte(mdTime)));
		}
		else {
			byte defValue = 2;
			writeChar('D', char(defValue));
		}
	}
	byte mdTimeOut = byte(readChar('d'));
	return mdTimeOut;
}

///////////////////// MD-Pin Active Time Remaining ////////////////////
byte ePIR::TimeLeft(word timeLeft){
	if (257 > timeLeft){
		if (256 > timeLeft){
			writeChar('O', char(lowByte(timeLeft)));
		}
		else {
			byte defValue = 2;
			writeChar('O', char(defValue));
		}
	}
	byte timeLeftOut = byte(readChar('o'));
	return timeLeftOut;
}

/////////////////////////// Unsolicited Mode //////////////////////////
char ePIR::Unsolicited(char unsolicited){
	if (unsolicited != '\0'){
		switch (unsolicited){
			case 'Y':
				writeChar('M', 'Y');
			break;
			default:
				writeChar('M', 'N');
			break;
		}
	}
	char unsolicitedOut = readChar('m');
	return unsolicitedOut;
}

//////////////////////////// Extended Range ///////////////////////////
char ePIR::Extended(char extended){
	if (extended != '\0'){
		switch (extended){
			case 'Y':
				writeChar('E', 'Y');
			break;
			default:
				writeChar('E', 'N');
			break;
		}
	}
	char extendedOut = readChar('e');
	return extendedOut;
}

////////////////////////// Frequency Response /////////////////////////
char ePIR::Frequency(char frequency){
	if (frequency != '\0'){
		switch (frequency){
			case 'H':
				writeChar('F', 'H');
			break;
			default:
				writeChar('F', 'L');
			break;
		}
	}
	char frequencyOut = readChar('f');
	return frequencyOut;
}

////////////////////// Suspend MD-Pin Activation //////////////////////
char ePIR::Suspend(char suspend){
	if (suspend != '\0'){
		switch (suspend){
			case 'y':
				writeChar('H', 'Y');
			break;
			default:
				writeChar('H', 'N');
			break;
		}
	}
	char suspendOut = readChar('h');
	return suspendOut;
}

///////////////////////////// Pulse Count /////////////////////////////
byte ePIR::PulseCount(byte pulseCount){
	if (0 < pulseCount){
		byte pcValue;
		switch (pulseCount){
			case 2:
				pcValue = 2;
				writeChar('P', '2');
			break;
			default:
				pcValue = 1;
				writeChar('P', '1');
			break;
		}
	}
	byte pulseCountOut = (byte(readChar('p')) - 48);
	return pulseCountOut;
}

///////////////////////////// Sensitivity /////////////////////////////
byte ePIR::Sensitivity(word sensitivity){
	if (257 > sensitivity){
		if (256 > sensitivity){
			writeChar('S', char(lowByte(sensitivity)));
		}
		else {
			byte defValue = 6;
			writeChar('S', char(defValue));
		}
	}
	byte sensitivityOut = byte(readChar('s'));
	return sensitivityOut;
}

///////////////////////// Detection Direction /////////////////////////
char ePIR::Direction(char direction){
	if (direction != '\0'){
		switch (direction){
			case '+':
				writeChar('V', '+');
			break;
			case '-':
				writeChar('V', '-');
			break;
			default:
				writeChar('V', 'A');
			break;
		}
	}
	char directionOut = readChar('v');
	return directionOut;
}

///////////////////////////// Reset ePIR //////////////////////////////
void ePIR::Reset(void){
	(*serialPtr).print('X');
	while ((*serialPtr).available() == 0);
	(*serialPtr).read();
	confirm();
	return;
}

/////////////////////////// ePIR Sleep Mode ///////////////////////////
void ePIR::Sleep(void){
	(*serialPtr).print('Z');
	while ((*serialPtr).available() == 0);
	(*serialPtr).read();
	confirm();
	return;
}

/////////////////////////// ePIR S/W Version //////////////////////////
word ePIR::Version(void){
	byte appVer;
	byte engVer;
	word version;

	(*serialPtr).print('i');
	while ((*serialPtr).available() == 0);
	appVer = (*serialPtr).read();
	engVer = (*serialPtr).read();
	version = word(appVer, engVer);
	return version;
}

ePIR EPIR = ePIR(); // ... Create one instance for user.