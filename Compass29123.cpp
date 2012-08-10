/* Compass29123
 *
 * Arduino Library for the Parallax 29123 Compass Module
 *
 * Written by Keith Rasmussen 8 June 2010
 * Released into the public domain.
 */

#include "WProgram.h"
#include "Compass29123.h"


// CTOR

Compass29123::Compass29123(unsigned int pin_din,		// Arduino pin number connected to 29123 Din pin
						 			unsigned int pin_dout,		// Arduino pin number connected to 29123 Dout pin
						 			unsigned int pin_enab,		// Arduino pin number connected to 29123 /EN pin
						 			unsigned int pin_clk,		// Arduino pin number connected to 29123 CLK pin
						 			unsigned int timeout_ms)	// Max time (ms.) for measurement before timeout.
{
	// Set Object Variables
	_pin_din = pin_din;
	_pin_dout = pin_dout;
	_pin_enab = pin_enab;
	_pin_clk = pin_clk;
	_timeout_ms = timeout_ms;
	
	_curStatus = MeasOK;		// set initial status
	
	// Set Arduino pins appropo
	pinMode(_pin_din, INPUT);   // Sets digital pin as input
   pinMode(_pin_dout, OUTPUT); // Sets digital pin as output
   pinMode(_pin_enab, OUTPUT); // Sets digital pin as output
   pinMode(_pin_clk, OUTPUT); // Sets digital pin as output

	#if 0
	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
   Serial.println("Compass29123: CTOR done.");
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */
	#endif
}

// getHeadingDeg
// 
//  returns:  0..359   curent heading in degrees
//            -1		 error in measurement (use getCurError() for type)
int Compass29123::getHeadingDeg()
{
	double x = 0;					// raw X value returned from measurement
	double y = 0;					// raw Y value returned from measurement
	double dirRad = 0;			// current direction in Radians
	double tmp = 0;				// working variable
	unsigned int dirDeg = 0; 	//current direction in Degrees
	
	_curStatus = getMeasurement(&x, &y);	// Get the measurement
	
	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
   Serial.print("Compass29123: Measurment status = 0x");
   Serial.print(_curStatus, HEX);
   Serial.print(", X = ");
   Serial.print(x, 4);
   Serial.print(", Y = ");
   Serial.println(y, 4);
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */
	
	if(_curStatus == MeasOK)
	{
		dirRad = atan2(-y, x);
   	tmp = (dirRad / M_PI) * 180;

 		if(tmp < 0)
  		{
    		tmp += 360;
  		}
  		dirDeg = (unsigned int) tmp;
  
		#ifdef Compass29123_Debug
		#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
   	Serial.print("Compass29123: dir (rads) = ");
		Serial.println(dirRad, 4);
		#endif /* if */
		#if (Compass29123_Debug & Compass29123_Debug_Level_High)
		Serial.print("Compass29123: dir (deg) = ");
   	Serial.println(dirDeg, DEC);
		#endif /* if */
		#endif /* ifdef Compass29123_Debug */
		
		return dirDeg;
	}
	else
	{
		#ifdef Compass29123_Debug
		#if (Compass29123_Debug & Compass29123_Debug_Level_High)
		Serial.print("Compass29123: Measurement Failed! status = 0x");
   	Serial.println(_curStatus, HEX);
		#endif /* if */
		#endif /* ifdef Compass29123_Debug */
		
		return(-1);
	}
}
		
		
// getCurError
//
//  returns error status as of last measurement
//		MeasOK = 0x00;  // Last measurement OK
//		MeasTO = 0x20;  // Last measurement timed out
//		MeasErr = 0x10; // Last measurement returned error MeasErr ORed with 4 bit error code from chip
		
byte Compass29123::getCurError()
{
	return _curStatus;
}
		
// getMeasurement
//
//  passed pointers to doubles for raw X and Y magnetic values to be stored
//  returns same values as returned by getCurError
//  if return is MeasOK *pX and *pY will have new measurement values
		
byte Compass29123::getMeasurement(double *pX,
			                 			 double *pY)
{
	// TODO unsigned long startTime = millis();  // save start time
	byte measStat = 0;

	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
	Serial.println("Compass29123: Into cmpGetMeasurement...");
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */


	reset();               // Start with Reset

	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
	Serial.println("Compass29123: reset complete...");
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */


	startMeasurement();    // Start Measurement

	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
	Serial.println("Compass29123: startMeasurement complete...");
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */


	delay(_timeout_ms);         // TODO delay for now, poll sooner in future

	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
	Serial.println("Compass29123: delay complete...");
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */


	measStat = getStatus();  // Get the raw status

	#ifdef Compass29123_Debug
	#if (Compass29123_Debug & Compass29123_Debug_Level_Low)
	Serial.print("Compass29123: getStatus returns 0x");
	Serial.println(measStat, HEX);
	#endif /* if */
	#endif /* ifdef Compass29123_Debug */

	if(measStat == StatusRdy)  
	{
		// Axis values ready, get 'em
	   *pX = getAxisValue();
	   *pY = getAxisValue();
	   _curStatus = MeasOK;
	}
	else
	{
		if((measStat & StatusRdy) == 0)
	   {
	   	// device not done...so it's a timeout
	      _curStatus = MeasTO;
	   }
	   else
	   {
			// Some other error return status as error ORed with the raw return code
	      _curStatus = measStat | MeasErr;
	   }
	}
	// Return the _Enable line to HIGH
	digitalWrite(_pin_enab, HIGH);  // /Enable HIGH

	//...and return with status
	return _curStatus;
}		

// reset
//
//  Reset the 29123 module
//  Assumes nothing
//  Leaves /EN HIGH
		
void Compass29123::reset()
{
	digitalWrite(_pin_enab, HIGH);  	// /Enable HIGH
	digitalWrite(_pin_clk, LOW);   	// Start with clk low
	digitalWrite(_pin_enab, LOW);   	// /Enable Low to start command
	cmdOut(CmdReset); 					// Send Reset
	digitalWrite(_pin_enab, HIGH);  	// /Enable HIGH to finish up
}
		
// startMeasurement
//
//  Send Start Measurement command to the 29123 module
//  Assumes  /EN HIGH on entry
//  Leaves /EN LOW on exit (in prep for status check)
		
void Compass29123::startMeasurement()
{
	digitalWrite(_pin_enab, LOW);   	// /Enable Low to start command
	cmdOut(CmdMeasure);					// Send Measure CMD
}
		
		
// getStatus
//
//  Get the measurement status following a startMeasurement command
//  Assumes startMeasurement() has been called and /EN is LOW
//  Returns 4 bit status value from chip
//          Leaves /EN LOW 
		
byte Compass29123::getStatus()
{
  byte stat = 0;
  byte mask = 0;

  // Pulse _Enable
  digitalWrite(_pin_enab, HIGH);
  digitalWrite(_pin_enab, LOW);

  cmdOut(CmdReport);

  // Get 4 bit Status
  for(int i = 0; i < 4; i++)
  {
    mask = 0x08 >> i;
    #ifdef DEBUG1
    Serial.print("Mask is = 0x");
    Serial.println(mask, HEX);
    #endif

    // Clock in a bit
    digitalWrite(_pin_clk, HIGH);
    digitalWrite(_pin_clk, LOW);
    // If data in is high, OR in the current mask
    if(digitalRead(_pin_din))
    {
      stat |= mask;
    }
  }
  return stat;
}
		
// cmdOut
//
// Shifts out a 4 bit command using _pin_clk and _pin_dout pins
// Shifts out MSB first
		
void Compass29123::cmdOut(byte cmd)
{
	digitalWrite(_pin_clk, HIGH);        // Clock out Bit 3
	digitalWrite(_pin_dout, cmd & 0x08);
	digitalWrite(_pin_clk, LOW);
	digitalWrite(_pin_clk, HIGH);        // Clock out Bit 2
	digitalWrite(_pin_dout, cmd & 0x04);
	digitalWrite(_pin_clk, LOW);
	digitalWrite(_pin_clk, HIGH);        // Clock out Bit 1
	digitalWrite(_pin_dout, cmd & 0x02);
	digitalWrite(_pin_clk, LOW);
	digitalWrite(_pin_clk, HIGH);        // Clock out Bit 0
	digitalWrite(_pin_dout, cmd & 0x01);
	digitalWrite(_pin_clk, LOW);
}
		
// getAxisValue
//
//  Reads 11 bit axis value from chip using _pin_clk and _pin_din pins
//  Ruturns a double with value read
		
double Compass29123::getAxisValue()
{
 	int val = 0;

 	for(int i = 0; i < 11; i++)
 	{
   	val = val << 1;
    	digitalWrite(_pin_clk, HIGH);
    	digitalWrite(_pin_clk, LOW);
    	// If data in is high, OR in the current mask
    	if(digitalRead(_pin_din))
    	{
      	val |= 1;
    	}
	}
	// if negative 11 bit num, make neg 16 bit
	if(val & MaskBit10)
	{
   	val |= MaskNeg11Bit;
  	}
  	return((double) val);
}
		