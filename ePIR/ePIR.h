/*
 *  ePIR.h created for ePIR library project on 09/01/2010 18:00:00.
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
*/

#include <WProgram.h>

#ifndef EPIR_H
#define EPIR_H

#define EPIR_VERSION 1.0 // ......................... The current ver/rev number of this library (This is the first release).

class ePIR { //////////////////////////////////////// /* **** Class 'ePIR' **** */
	private: ////////////////////////////////////////// /* Private Functions. */
		char readChar(char); // ......................... Reads values from the ePIR.
		char writeChar(char, char); // .................. Writes values to the ePIR.
		char confirm(void); // .......................... Sends confirmation command for special functions 'Sleep' and 'Reset'.
	public: /////////////////////////////////////////// /* Public Functions */
		ePIR();
		~ePIR();
//////// Return type / Funtion name / Argument(s) ... Valid return/argument values. ////////
		void Init(byte, byte, byte); // ................. <1,2,3> <Any unused Arduino pin> <Any unused Arduino pin>
		void Init(byte);   // Added, AR - slimmed-down version
		char Status(void); // ........................... READ ONLY <Y,N,U>
		byte LightLevel(void); // ....................... READ ONLY <0-255>
		byte GateThresh(word threshold = 257); // ....... <0-255/256 sets default>
		char MDRmode(char mdrMode = '\0'); // ........... <M,R>
		byte MDtime(word mdTime = 257); // .............. <0-255/256 sets default>
		byte TimeLeft(word timeLeft = 257); // .......... <0-255/256 sets default>
		char Unsolicited(char unsolicited = '\0'); // ... <Y,N>
		char Extended(char extended = '\0'); // ......... <Y,N>
		char Frequency(char frequency = '\0'); // ....... <H,L>
		char Suspend(char suspend = '\0'); // ........... <Y,N>
		byte PulseCount(byte pulseCount = 0); // ........ <1,2>
		byte Sensitivity(word sensitivity = 257); // .... <0-255/256 sets default>
		char Direction(char direction = '\0'); // ....... <A,+,->
		void Reset(void); // ............................ Returns ACK if successful/NACK if not.
		void Sleep(void); // ............................ Returns ACK if successful/NACK if not.
		word Version(void); // .......................... READ ONLY val1(highByte)<0-255> / val2(lowByte)<0-255>
};

extern ePIR EPIR; // Sets up an instance name of 'EPIR'.
#endif