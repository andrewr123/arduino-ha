
// MCP23S17 SPI 16-bit IO expander
// http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf

/*
    Version history
    ---------------
  
    v1 - Mar 11 - dreamcat https://github.com/dreamcat4/Mcp23s17
    v2 - Oct 11 - Andrew Richards - https://github.com/andrewr123/arduino-ha
	 Modifications to add interrupt handling and allow use with standard SPI library

    Licensing (for v2 code; no licensing specified for v1 code)
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


#ifndef Mcp23s17_h
#define Mcp23s17_h

//#define slave_select_pin 30     // Use of builtin constant enables fast digitalWrite

// ********************  CONSTANTS  *********************

// Public Constants
const static uint8_t GPIO_A0 = 0;
const static uint8_t GPIO_A1 = 1;
const static uint8_t GPIO_A2 = 2;
const static uint8_t GPIO_A3 = 3;
const static uint8_t GPIO_A4 = 4;
const static uint8_t GPIO_A5 = 5;
const static uint8_t GPIO_A6 = 6;
const static uint8_t GPIO_A7 = 7;
const static uint8_t GPIO_B0 = 8;
const static uint8_t GPIO_B1 = 9;
const static uint8_t GPIO_B2 = 10;
const static uint8_t GPIO_B3 = 11;
const static uint8_t GPIO_B4 = 12;
const static uint8_t GPIO_B5 = 13;
const static uint8_t GPIO_B6 = 14;
const static uint8_t GPIO_B7 = 15;

// Values for IOCMode = corresponding bit in INTCON
const static uint8_t IOCPREV = 0x00;	// Compare pin against previous value to identify interrupt
const static uint8_t IOCDEFVAL = 0x01;	// Compare pin against (opposite of) DEFVAL to identify interrupt
const static uint8_t ONCHANGE = 1;		// INTCON == 0: raise interrupt on any change
const static uint8_t FALLEN = 2;		// INTCON == 0 plus interpretation: raise interrupt if low after high
const static uint8_t RISEN = 3;			// INTCON == 0 plus interpretation: raise interrupt if high after low
const static uint8_t WHILEHIGH = 4;		// INTCON == 1: raise interrupt if high (DEFVAL == 0)
const static uint8_t WHILELOW = 5;		// INTCON == 1: raise interrupt if low (DEFVAL == 1)
  
const static uint8_t DISABLED = 0x00;
const static uint8_t ENABLED  = 0x01;

// ******************* CLASS *************************

class MCP23S17
{
  public:
    // You must specify the slave select pin
    void begin(uint8_t slave_select);
    // Optionally, up to 8 devices can share the same slave select pin
    void begin(uint8_t slave_select, byte aaa_hw_addr);
    // AR - Setup function with interrupts
    void beginInt(uint8_t slave_select, uint8_t intMode);
 	void beginInt(uint8_t slave_select, uint8_t intMode, byte aaa_hw_addr);

    // GPIO 16-bit Combined Port (GPIO A0-A7 + GPIO B0-B7)
    void pinMode(bool mode);
    void port(uint16_t value);
    uint16_t port();

    // Work on individual pins 0-15
    void pinMode(uint8_t pin, bool mode);
    void digitalWrite(uint8_t pin, bool value);
    int  digitalRead(uint8_t pin);
    void intMode(byte pin, byte IOCMode);
    void intEnable(byte pin);
    void intDisable(byte pin);
    void intDisable();

    boolean intFlag (byte pin);
    uint16_t intFlag ();
    uint16_t intCapture ();
    uint16_t intInterpret ();
    uint16_t intValid ();

    uint16_t read_addr(byte addr);
    uint16_t debug(byte addr);

  protected:
    // Protected Constants

    const static uint8_t IOCONA = 0x0A;   	// Config register is here on power up
    const static uint8_t IOCON  = IOCONA; 	// Config register is here on power up

    // Config options
    const static uint8_t BANK   = B10000000;	// Best not to set - re-maps all addresses    
    const static uint8_t MIRROR = B01000000; 	// Mirror interrupts on both INTA and INTB    
    const static uint8_t SEQOP  = B00100000; 	// Not needed if BANK=0 (default)
    const static uint8_t DISSLW = B00010000;	// Slew rate for SDA output (N/A for SPI chip?)
    const static uint8_t HAEN   = B00001000; 	// Enable the AAA 3-bit chip select
    const static uint8_t ODR	= B00000100;	// Configure INT as open drain output
    const static uint8_t INTPOL	= B00000010;	// INT pin polarity: 1 = active high

    // As BANK=0, Register addresses are therefore mapped per
    // "TABLE 1-6:  CONTROL REGISTER SUMMARY (IOCON.BANK = 0)"

    const static uint8_t IODIRA = 0x00;
    const static uint8_t IODIRB = 0x01;
    const static uint8_t IODIR  = IODIRA;

    const static uint8_t GPINTEN = 0x04;  	// Interrupt enable pins

    const static uint8_t DEFVAL  = 0x06;  	// If GPINTEN and IOCDEF then interrupt on opposite

    const static uint8_t INTCON  = 0x08;	// How to identify interrupts - IOCDEF or IOCPREV

    const static uint8_t GPPUA  = 0x0C;
    const static uint8_t GPPUB  = 0x0D;
    const static uint8_t GPPU   = GPPUA;

    const static uint8_t INTF	= 0x0e;		// Interrupt flags

    const static uint8_t INTCAP = 0x10;		// Interrupt values on capture

    const static uint8_t GPIOA  = 0x12;
    const static uint8_t GPIOB  = 0x13;
    const static uint8_t GPIO   = GPIOA;

    #ifndef slave_select_pin
    	uint8_t slave_select_pin;
    #endif
    uint16_t intTest;			// 0 = no interpretation (accept interrupt), 1 = interpet by comparing INTCAP against intDirection
    uint16_t intDirection;		// Set in intEnable, then (if intTest == 1) used to interpret INTCAP when IOCMode == RISING (1) or FALLING (0)
    uint16_t intEnablePins;		// Set/unset in intEnable/intDisable; amended in intValid.  Copy of GPINTEN

    byte aaa_hw_addr;

    byte read_cmd;
    byte write_cmd;

    void setup_ss(uint8_t slave_select);
    void setup_device(uint8_t aaa_hw_addr);
    void raiseInterruptWith (byte mode);

    void write_addr(byte addr, uint16_t data);
    void write_IOCON(byte data);

    uint16_t byte2uint16(byte high_byte, byte low_byte);
    byte uint16_high_byte(uint16_t uint16);
    byte uint16_low_byte(uint16_t uint16);
};

#endif // Mcp23s17_h



