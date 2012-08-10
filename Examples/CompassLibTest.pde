#include <Compass29123.h>

// Test Sketch for Compass29123 Library
//
// Keith Rasmussen
// Isotropic Laboratories
// No Rights Reserved


#define DEBUG

#define LED         13  // "North" LED connected to pin 13
#define CMP_D_IN     2  // Digital Pin 2 for data input from compass
#define CMP_D_OUT    3  // Digital Pin 3 for data output to compass
#define _CMP_EN_OUT  4  // Digital Pin 4 for Enable output to compass
#define CMP_CLK_OUT  5  // Digital Pin 5 for Clk output to compass
#define CMP_PORT_LIM    345      // Port limit for LED display = 345 deg.
#define CMP_STBD_LIM    015      // Starboard limit for LED display = 015 deg.

unsigned int heading = 0;  // current direction in degrees

Compass29123 compass(CMP_D_IN,
                     CMP_D_OUT,
                     _CMP_EN_OUT,
                     CMP_CLK_OUT);

void setup()
{
  pinMode(LED, OUTPUT);       // Sets digital pin as output
  Serial.begin(9600);         // open serial port to send data back to PC
  digitalWrite(LED, LOW);     // Turn off LED
  #ifdef DEBUG
  Serial.println("CompassLibTest: Setup done.");
  #endif
}

void loop()
{
  // OK, take the measurement
  int newHeading = compass.getHeadingDeg();
  
  if(newHeading != -1)  // Check for a valid reading...
  {
     heading = (unsigned int) newHeading;
     
     #ifdef DEBUG
     Serial.print("CompassLibTest: heading (deg) = ");
     Serial.println(heading, DEC);
     #endif
  }
  else
  {
     // Error in reading Measurement
     #ifdef DEBUG
     Serial.print("CompassLibTest: Error = 0x");
     Serial.println(compass.getCurError(), HEX);
     #endif
  }
  
  // Display Heading
  compassDisplay(heading);
}

void compassDisplay(unsigned int cmpDir)
{
  if(CMP_PORT_LIM > CMP_STBD_LIM)
  {
    if((cmpDir >= CMP_PORT_LIM) || (cmpDir <= CMP_STBD_LIM))
    {
      // LED ON
        digitalWrite(LED, HIGH);  // Turn on LED
    }
    else
    {
        digitalWrite(LED, LOW);  // Turn off LED
    }
  }
  else
  {
    if((cmpDir >= CMP_PORT_LIM) && (cmpDir <= CMP_STBD_LIM))
    {
      // LED ON
        digitalWrite(LED, HIGH);  // Turn on LED
    }
    else
    {
        digitalWrite(LED, LOW);  // Turn off LED
    }
  }
}


