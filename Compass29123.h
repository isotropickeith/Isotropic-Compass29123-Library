/* Compass29123
 *
 * Arduino Library for the Parallax 29123 Compass Module
 *
 * Written by Keith Rasmussen 7 June 2010
 * Released into the public domain.
 */

#ifndef Compass29123_h
#define Compass29123_h

// Debug level defines
//#define Compass29123_Debug_Level_High 0x0001
//#define Compass29123_Debug_Level_Low  0x0002

// This define...if uncommented will generate code for debug messages
// If defined, value is ORed Debug level flags
// Assumes Serial.begin() called in main setup()
#define Compass29123_Debug (Compass29123_Debug_Level_High | Compass29123_Debug_Level_Low)

#include "WProgram.h"
#include <math.h>


class Compass29123
{
	public:
		// CTOR
		static const unsigned int TimeoutDefaultMs = 50;		// Default timeout value for measurement		
		
		Compass29123(unsigned int pin_din,				// Arduino pin number connected to 29123 Din pin
						 unsigned int pin_dout,				// Arduino pin number connected to 29123 Dout pin
						 unsigned int pin_enab,				// Arduino pin number connected to 29123 /EN pin
						 unsigned int pin_clk,				// Arduino pin number connected to 29123 CLK pin
						 unsigned int timeout_ms = TimeoutDefaultMs);	// Max time (ms.) for measurement before timeout.
		// getHeadingDeg
		// 
		//  returns:  0..359   curent heading in degrees
		//            -1		 error in measurement (use getCurError() for type)
		int getHeadingDeg();
		
		
		// getCurError
		//
		//  returns error status as of last measurement
		static const byte MeasOK = 0x00;  // Last measurement OK
		static const byte MeasTO = 0x20;  // Last measurement timed out
		static const byte MeasErr = 0x10; // Last measurement returned error MeasErr ORed with 4 bit error code from chip
		
		byte getCurError();
		
		// getMeasurement
		//
		//  passed pointers to doubles for raw X and Y magnetic values to be stored
		//  returns same values as returned by getCurError
		//  if return is MeasOK *pX and *pY will have new measurement values
		
		byte getMeasurement(double *pX,
			                 double *pY);
			
	private:
		
		static const byte CmdReset   = 0x00; 				// 4 bit Reset Command
		static const byte CmdMeasure = 0x08; 				// 4 bit Measure Command
		static const byte CmdReport  = 0x0C;					// 4 bit Report Command
		static const byte StatusRdy  = 0x0C;					// 4 bit Ready Status
		static const unsigned int MaskBit10	  = 0x0400;	// Mask for Bit 10, used to test for negative 11 bit number
		static const unsigned int MaskNeg11Bit = 0xF800; // Mask to OR in to make neg 11 bit number into 16 bit
		
		unsigned int _pin_din;		// Arduino pin number connected to 29123 Din pin
		unsigned int _pin_dout;		// Arduino pin number connected to 29123 Dout pin
		unsigned int _pin_enab;		// Arduino pin number connected to 29123 /EN pin
		unsigned int _pin_clk;		// Arduino pin number connected to 29123 CLK pin
		unsigned int _timeout_ms;	// Max time (ms.) for measurement before timeout
		
		byte	_curStatus;				// Last Status from measurement
		

		// reset
		//
		//  Reset the 29123 module
		//  Assumes nothing
		//  Leaves /EN HIGH
		
		void reset();
		
		// startMeasurement
		//
		//  Send Start Measurement command to the 29123 module
		//  Assumes  /EN HIGH on entry
		//  Leaves /EN LOW on exit (in prep for status check)
		
		void startMeasurement();
		
		
		// getStatus
		//
		//  Get the measurement status following a startMeasurement command
		//  Assumes startMeasurement() has been called and /EN is LOW
		//  Returns 4 bit status value from chip
		//          Leaves /EN LOW 
		
		byte getStatus();
		
		// cmdOut
		//
		// Shifts out a 4 bit command using _pin_clk and _pin_dout pins
		// Shifts out MSB first
		
		void cmdOut(byte cmd);
		
		// getAxisValue
		//
		//  Reads 11 bit axis value from chip using _pin_clk and _pin_din pins
		//  Ruturns a double with value read
		
		double getAxisValue();
		
};

#endif /* Compass29123_h */