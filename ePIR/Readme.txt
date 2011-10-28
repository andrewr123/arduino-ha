ePIR.txt created for ePIR library project on 09/01/2010 18:00:00.
Written using AVR Project IDE v1.97.

	This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

	This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
MA 02110-1301, USA.

* * * * * * * * * * * * * * * * * * * *
* *  Code written by Corey Johnson  * *
* *  September 1, 2010              * *
* * * * * * * * * * * * * * * * * * * *

	This is a library for the 'Zilog ePIR Motion Detection -Zdots SBC(Single Board Computer)'. This library was 
written for the Arduino Mega. I'm sure with a little work it can be ported to other boards. However, I have
no current plans to do so. The example program shows some various uses of the library and is well commented.
For hardware configuration see the example sketch that came with the library or go to the Zilog website 
for documentation about the hardware. This is the first working library I've ever written so if you have any
comments or suggestions send an email to trlrtrsh2@cox.net. All I ask is that if you make any improvements
to this library please give credit to the original author(Me), give the library a new version/revision number,
and please send me a copy so that I may learn from it. I am not an expert at programming, but I'm always eager
to learn new things. I hope you find this library useful. Enjoy!  ;o)

******************************************* Functions and Syntax *******************************************

*** Initialize Arduino/ePIR port/pins
EPIR.Init(SerialPort, MD/Rpin, SLPpin); - arguments: byte[1, 2, 3] / byte[any unused pin] / byte[any unused pin]  **(WRITE ONLY) 

*** Motion Detected Status
value(char) = EPIR.Status(); - returns: [Y, N, U]  **(READ ONLY)

*** Light Gate Level
value(byte) = EPIR.LightLevel(); - returns: [0-255]  **(READ ONLY)

***Light Gate Threshold
value(byte) = EPIR.GateThresh(word value); - returns: [0-255] - arguments: [0-255, 256, 257-65535]

*** MD/R Pin Mode
value(char) = EPIR.MDRmode(char value); - returns: [R, M] - arguments: [none, '*', 'R', 'M']

*** MD-Pin Active Time
byte ePIR::MDtime(word value); - returns: [0-255] - arguments: [0-255, 256, 257-65535] *see zilog documentation.

*** MD-Pin Active Time Remaining
value(byte) = EPIR.TimeLeft(word value); - returns: [0-255] - arguments: [0-255, 256, 257-65535] *see zilog documentation.

*** Unsolicited Mode
value(char) = EPIR.Unsolicited(char value); - returns: [Y, N] - arguments: [none, '*', 'Y', 'N']

*** Extended Range
value(char) = EPIR.Extended(char value); - returns: [Y, N] - arguments: [none, '*', 'Y', 'N']

*** Frequency Response
value(char) = EPIR.Frequency(char value); - returns: [L, H] - arguments: [none, '*', 'L', 'H']

*** Suspend MD-Pin Activation
value(char) = EPIR.Suspend(char value); - returns: [Y, N] - arguments: [none, '*', 'Y', 'N']

*** Pulse Count
value(byte) = EPIR.PulseCount(byte value); - returns: [1, 2] - arguments: [0, 1, 2, 3-255]

*** Sensitivity
value(byte) = EPIR.Sensitivity(word value); - returns: [0-255] - arguments: [0-255, 256, 257-65535]

*** Detection Direction
value(char) = EPIR.Direction(char value); - returns: [A, +, -] - arguments: [none, '*', 'A', '+', '-']

*** Reset ePIR
EPIR.Reset(); - Resets the device to all default values!  *see zilog documentation.

*** ePIR Sleep Mode
EPIR.Sleep(); - Puts the device into "Sleep" mode.  *see zilog documentation.

*** ePIR S/W Version
value(word) = EPIR.Version(); - returns: highByte[0-255], lowByte[0-255]  *see zilog documentation.

--------------------------------------------------------------------------------------------------------------------------

("none" means no value is given to the function, e.g. EPIR.function();)
('*' means an invalid character is given to the function)

***** see zilog's documentation for description and use of the functions used in this library *****
(note: The function names used here are very similar to zilogs product spec. sheet names)