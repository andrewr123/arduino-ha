
// MCP23S17 Spi 16-bit IO expander
// http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf

// For the cmd, AAA is the 3-bit MCP23S17 device hardware address.
// Useful for letting up to 8 chips sharing same Spi Chip select
// #define MCP23S17_READ  B0100AAA1 
// #define MCP23S17_WRITE B0100AAA0 

// Assumes prior initialisation of SPI comms
// For speed, direct access to SPI registers used in place of standard SPI.transfer method

#include "WConstants.h"
#include "Mcp23s17.h"
#include "wiring_private.h"



//---------- public ----------------------------------------------------

#ifdef slave_select_pin
void MCP23S17::begin(uint8_t dummy)
#else
void MCP23S17::begin(uint8_t slave_select_pin)
#endif
{
  	setup_ss(slave_select_pin);
  	setup_device(0x00);
}

#ifdef slave_select_pin
void MCP23S17::begin(uint8_t dummy, byte aaa_hw_addr)
#else
void MCP23S17::begin(uint8_t slave_select_pin, byte aaa_hw_addr)
#endif
{

  setup_ss(slave_select_pin);

  // Set the aaa hardware address for this chip by tying pins A0, A1, and A2 to either 5v or GND.
  // We enable HAEN on all connected devices before we can address them individually
  // NB: this will set the HAEN bit for device 0x00 as well as any unaddressed device(s)
  setup_device(0x00);
  write_IOCON((byte)read_addr(IOCON) | HAEN);

  // Remember the hardware address for this chip
  setup_device(aaa_hw_addr);
}

#ifdef slave_select_pin
void MCP23S17::beginInt(uint8_t dummy, uint8_t intMode)
#else
void MCP23S17::beginInt(uint8_t slave_select_pin, uint8_t intMode)
#endif
{
  	begin(slave_select_pin);
	raiseInterruptWith(intMode);
  	intTest = 0;
  	intDirection = 0;
  	intEnablePins = 0;
}

#ifdef slave_select_pin
void MCP23S17::beginInt(uint8_t dummy, uint8_t intMode, byte aaa_hw_addr)
#else
void MCP23S17::beginInt(uint8_t slave_select_pin, uint8_t intMode, byte aaa_hw_addr)
#endif
{
  	begin(slave_select_pin, aaa_hw_addr);
	raiseInterruptWith(intMode);
  	intTest = 0;
  	intDirection = 0;
  	intEnablePins = 0;
}

void MCP23S17::pinMode(bool mode)
{
  uint16_t input_pins;
  if(mode == INPUT)
    input_pins = 0xFFFF;
  else
    input_pins = 0x0000;

  write_addr(IODIR, input_pins);
}

void MCP23S17::port(uint16_t value)
{
  write_addr(GPIO,value);
}

uint16_t MCP23S17::port()
{
  return read_addr(GPIO);
}

void MCP23S17::pinMode(uint8_t pin, bool mode)
{
  if(mode == INPUT)
    write_addr(IODIR, read_addr(IODIR) | 1<<pin );
  else
    write_addr(IODIR, read_addr(IODIR) & ~(1<<pin) );
}

void MCP23S17::digitalWrite(uint8_t pin, bool value)
{
  if(value)
    write_addr(GPIO, read_addr(GPIO) | 1<<pin );  
  else
    write_addr(GPIO, read_addr(GPIO) & ~(1<<pin) );  
}

int MCP23S17::digitalRead(uint8_t pin)
{
  return (int)(read_addr(GPIO) & 1<<pin);
}



void MCP23S17::intMode (byte pin, byte IOCMode) {
	pinMode(pin, INPUT);			// Set up this pin as an input
	
	switch (IOCMode) {				// Set appropriate control flags in MCP and (if RISING/FALLING) locally
		case ONCHANGE:
			write_addr (INTCON, read_addr(INTCON) & ~(1 << pin));	// IOCPREV
			intTest &= ~(1 << pin);		// No test for direction needed
			break;
		case RISING:
			write_addr (INTCON, read_addr(INTCON) & ~(1 << pin));	// IOCPREV
			intTest |= (1 << pin);		// Test for direction against INTCAP needed after interrupt
			intDirection |= (1 << pin);		// Was rising change if INTCAP is set
			break;
		case FALLING:
			write_addr (INTCON, read_addr(INTCON) & ~(1 << pin));	// IOCPREV
			intTest |= (1 << pin);		// Test for direction against INTCAP needed after interrupt
			intDirection &= ~(1 << pin);	// Was falling change if INTCAP is clear
			break;
		case WHILEHIGH:
			write_addr (INTCON, read_addr(INTCON) | (1 << pin));	// IOCDEFVAL
			write_addr (DEFVAL, read_addr(DEFVAL) & ~(1 << pin));	// Interrupt if opposite to 0
			intTest &= ~(1 << pin);		// No test for direction needed
			break;
		case WHILELOW:
			write_addr (INTCON, read_addr(INTCON) | (1 << pin));		// IOCDEFVAL
			write_addr (DEFVAL, read_addr(DEFVAL) | (1 << pin));		// Interrupt if opposite to 1
			intTest &= ~(1 << pin);		// No test for direction needed
			break;
	}
}

void MCP23S17::intEnable (byte pin) {
	intEnablePins |= 1 << pin;
	write_addr (GPINTEN, read_addr(GPINTEN) | 1 << pin);		// Results in instant interrupt if enabling into int condition
																// See section 1.7.5 of data sheet
}

void MCP23S17::intDisable (byte pin) {
	write_addr (GPINTEN, read_addr (GPINTEN) & ~(1 << pin));
	intEnablePins &= ~(1 << pin);
}

void MCP23S17::intDisable () {
	write_addr (GPINTEN, 0x0000);
	intEnablePins = 0;
}

boolean MCP23S17::intFlag (byte pin) {
  return (bool)(read_addr (INTF) & 1 << pin);
}

uint16_t MCP23S17::intFlag () {
  return read_addr (INTF);
}

uint16_t MCP23S17::intCapture () {
  return read_addr (INTCAP);
}

uint16_t MCP23S17::intValid () {   	// Optimised function to interpret interrupts on behalf of ISR
					// Returns (for each pin) 1 for valid interrupt, 0 if invalid (condition not met)
					// Pins with valid interrupt have interrupts disabled; remainder of pins unchanged
					// If slave_select_pin declared as inbuild constant vs passed as variable, then takes:
					// ~7uS vs ~ 10uS to test int flags and return if null
					// ~25uS vs ~40uS to process interrupts
					
  uint16_t flagBits;		// Copy of INTF register interpreted using intDirection against INTCAP if intTest set
  uint16_t intCaptured;		// Indicates what caused the interrupt - copy of INTCAP register
  uint16_t intDirOK;		// 1 indicates change was same as desired direction (High - Low, or Low - High)
  uint16_t intIgnore;		// 1 if the interrupt should be ignored

  byte low_byte;
  byte high_byte;
  uint8_t oldSREG;
  
  oldSREG = SREG;
  cli();			// Protect following from interrupts (if not already within ISR)
  
  // Find out which pins triggered the interrupt - inline version of read_addr(INTF);

  ::digitalWrite(slave_select_pin, LOW);

  SPDR = read_cmd; while (!(SPSR & (1<<SPIF)));
  SPDR = INTF; while (!(SPSR & (1<<SPIF)));
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); low_byte  = SPDR;
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); high_byte = SPDR;
  flagBits = (uint16_t)high_byte<<8 | (uint16_t)low_byte;
  
  ::digitalWrite(slave_select_pin, HIGH);
  
  // In a multi-chip environment (with interrupt out lines commoned) then this chip might not
  // have raised the interrupt - in which case can quit early
  
  if (flagBits == 0) {
	SREG = oldSREG;		// Restore previous interrupt state
    return flagBits;
  }

  // Need to disable interrupts on valid int in pin(s).  But slightly complicated by modes . . 
  // If WHILEHIGH, WHILELOW or ONCHANGE then interrupt is unconditional (so intTest == 0), so
  // can disable interrupting pin(s) immediately.  (NB: This also ensures that WHILEHIGH and WHILELOW 
  // pins are disabled before reading INTCAP to ensure interrupt out is cleared - otherwise int in 
  // condition could remain valid and int out is not cleared)
  // If FALLEN or RISEN then need to read INTCAP before deciding if int in is valid, so skip write to GPINTEN here
  // NB: If multiple interrupts then possibility that have a mix of conditional and unconditional interrupts in,
  // in which case accept inefficiency of two writes to GPINTEN (likely to be very rare - requires multiple interrupts to 
  // arrive before master ISR gets round to handling the int out, and requires int in conditions to differ)
  
  if ((intTest & flagBits) == 0)  {	    // TRUE if all interrupts are unconditional
	intEnablePins ^= flagBits;			// Clear pins that interrupted
	
	::digitalWrite(slave_select_pin, LOW);

    SPDR = write_cmd; while (!(SPSR & (1<<SPIF)));
    SPDR = GPINTEN; while (!(SPSR & (1<<SPIF)));
    SPDR = (byte)(intEnablePins & 0x00ff); while (!(SPSR & (1<<SPIF))); 
    SPDR = (byte)(intEnablePins >> 8); while (!(SPSR & (1<<SPIF))); 

    ::digitalWrite(slave_select_pin, HIGH);
  }

  // Clear the interrupt out (and get values for any conditional tests).  Inline version of read_addr (INTCAP)

  ::digitalWrite(slave_select_pin, LOW);

  SPDR = read_cmd; while (!(SPSR & (1<<SPIF)));
  SPDR = INTCAP; while (!(SPSR & (1<<SPIF)));
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); low_byte  = SPDR;
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); high_byte = SPDR;
  intCaptured = (uint16_t)high_byte<<8 | (uint16_t)low_byte; 
    
  ::digitalWrite(slave_select_pin, HIGH);
    
  // If any conditional interrupts, then do the tests and disable the pins that caused a valid interrupt, 
  // Tests are a combination of:
  // - intTest to identify pins needing a direction test (only set if FALLEN or RISEN)
  // - intDirection to identify whether FALLEN (0) or RISEN (1)
  // - intCapture to identify the actual reading causing the interrupt  
  
  if (intTest & flagBits) {
	  intDirOK = (intDirection & intCaptured) | (~intDirection & ~intCaptured);  // Bitwise 1 if target and actual direction the same
	  intIgnore = intTest & ~intDirOK;											 // Bitwise 1 if direction test was needed but failed
	  flagBits ^= (intIgnore & flagBits);										 // Bitwise 1 if valid interrupt 
	  
	  intEnablePins ^= flagBits;												 // Bitwise 1 if no interrupt or not valid

	  ::digitalWrite(slave_select_pin, LOW);
	
	  SPDR = write_cmd; while (!(SPSR & (1<<SPIF)));
	  SPDR = GPINTEN; while (!(SPSR & (1<<SPIF)));
	  SPDR = (byte)(intEnablePins & 0x00ff); while (!(SPSR & (1<<SPIF))); 
	  SPDR = (byte)(intEnablePins >> 8); while (!(SPSR & (1<<SPIF))); 
	
	  ::digitalWrite(slave_select_pin, HIGH); 
  }
  
  SREG = oldSREG;		// Restore previous interrupt state
  
  return flagBits;
}


//------------------ protected -----------------------------------------------

uint16_t MCP23S17::byte2uint16(byte high_byte, byte low_byte)
{
  return (uint16_t)high_byte<<8 | (uint16_t)low_byte;
}

byte MCP23S17::uint16_high_byte(uint16_t uint16)
{
  return (byte)(uint16>>8);
}

byte MCP23S17::uint16_low_byte(uint16_t uint16)
{
  return (byte)(uint16 & 0x00FF);
}

#ifdef slave_select_pin
void MCP23S17::setup_ss(uint8_t dummy)
{
  // Set slave select (Chip Select) pin for Spi Bus, and start high (disabled)
  ::pinMode(slave_select_pin,OUTPUT);
  ::digitalWrite(slave_select_pin,HIGH);
}
#else
void MCP23S17::setup_ss(uint8_t slave_select_pin)
{
  // Set slave select (Chip Select) pin for Spi Bus, and start high (disabled)
  ::pinMode(slave_select_pin,OUTPUT);
  ::digitalWrite(slave_select_pin,HIGH);
  this->slave_select_pin = slave_select_pin;
}
#endif


void MCP23S17::setup_device(uint8_t aaa_hw_addr)
{
  this->aaa_hw_addr = aaa_hw_addr;
  this->read_cmd  = B01000000 | aaa_hw_addr<<1 | 1<<0; // MCP23S17_READ  = B0100AAA1 
  this->write_cmd = B01000000 | aaa_hw_addr<<1 | 0<<0; // MCP23S17_WRITE = B0100AAA0
}


void MCP23S17::raiseInterruptWith (byte mode) {
  	if (mode == HIGH) {
	  	write_IOCON (read_addr(IOCON) | MIRROR | INTPOL);
  	}
    else {
	    write_IOCON ( (read_addr(IOCON) | MIRROR) & ~INTPOL);
    }
}


uint16_t MCP23S17::read_addr(byte addr)
{
  byte low_byte;
  byte high_byte;
  uint8_t oldSREG;

  oldSREG = SREG;
  cli();			// Protect following from interrupts

  ::digitalWrite(slave_select_pin, LOW);

  SPDR = read_cmd; while (!(SPSR & (1<<SPIF)));
  SPDR = addr; while (!(SPSR & (1<<SPIF)));
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); low_byte  = SPDR;
  SPDR = 0x00; while (!(SPSR & (1<<SPIF))); high_byte = SPDR;

  ::digitalWrite(slave_select_pin, HIGH);

  SREG = oldSREG;		// Restore previous interrupt state

  return byte2uint16(high_byte,low_byte);
}



void MCP23S17::write_addr(byte addr, uint16_t data)
{
  uint8_t oldSREG;

  oldSREG = SREG;
  cli();

  ::digitalWrite(slave_select_pin, LOW);

  SPDR = write_cmd; while (!(SPSR & (1<<SPIF)));
  SPDR = addr; while (!(SPSR & (1<<SPIF)));
  SPDR = uint16_low_byte(data); while (!(SPSR & (1<<SPIF)));
  SPDR = uint16_high_byte(data); while (!(SPSR & (1<<SPIF)));

  ::digitalWrite(slave_select_pin, HIGH);

  SREG = oldSREG;		// Restore previous interrupt state

}


void MCP23S17::write_IOCON(byte data)
{
  uint8_t oldSREG;

  oldSREG = SREG;
  cli();

  ::digitalWrite(slave_select_pin, LOW);

  SPDR = write_cmd; while (!(SPSR & (1<<SPIF)));
  SPDR = IOCON; while (!(SPSR & (1<<SPIF)));
  SPDR = data; while (!(SPSR & (1<<SPIF)));

  ::digitalWrite(slave_select_pin, HIGH);

  SREG = oldSREG;		// Restore previous interrupt state

}
