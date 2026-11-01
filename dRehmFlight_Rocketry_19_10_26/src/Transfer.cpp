/* Transfer data from the Teensy to another nearby microcontroller over I2C wires.
   Sends data on Wire1() or Wire2() while the main loop is running (IMU's and BMPXXX run on Wire() - i.e. Wire0 )
   Data is sent as a series of bytes.

   This version sends IMU data to a very nearby ESP32 for analysis - keep the I2C wires really short!

   Rick Newlands April 2026
*/
#include <Arduino.h>
#include <Wire.h>                 // I2C communication
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands


// externals
extern Euler_angles_degrees horizontal_IMU; // vertical_IMU;

// define the slave I2C address
const uint8_t slave_address = 64;


void Transferacross_setup(void)
{
  // Teensy pin values:
  // As connected: SDA (yellow), SCL (blue)
  // Wire  uses the pins SDA  18, SCL  19
  // Wire1 uses the pins SDA1 17, SCL1 16
  // Wire2 uses the pins SDA2 25, SCL2 24
  Wire1.begin(); // Wire2.begin();
}

void Transferacross_loop(void)
{
  /* send new data to slave microcontroller
  Serial.println("sending data");

  Serial.print(F(" h_pitch:"));
  Serial.print(horizontal_IMU.pitch_degs);

  Serial.print(F("h_roll:"));
  Serial.println(horizontal_IMU.roll_degs);
  */

  // Convert floats into 16-bit integers
  int16_t a = (int) (horizontal_IMU.pitch_degs);
  int16_t b = (int) (horizontal_IMU.roll_degs);

  /* Then break the 16-bit integers into a series of 8-bit integers (bytes) 
     and send to the Slave microcontroller over Wire1 (or change to Wire2) */
  Wire1.beginTransmission(slave_address);
  Wire1.write(reinterpret_cast<uint8_t *>(&a), sizeof(a)); // convert to a series of bytes
  Wire1.write(reinterpret_cast<uint8_t *>(&b), sizeof(b)); // convert to a series of bytes
  Wire1.endTransmission();
}


/*******************************************
 
// Copy the following code to the slave microcontroller (e.g. an ESP32):
// Rick Newlands March 2026


#include <Arduino.h>
#include <Wire.h>    // I2C

// define slave I2C address (must be same as above!)
const uint8_t slave_address = 64;

int16_t Pitch_degs;
int16_t Roll_degs;


// Receive bytes (8-bit integers) and convert back into 16-bit integers
void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    Pitch_degs = (Wire.read() | Wire.read()<<8 );
    Roll_degs  = (Wire.read() | Wire.read()<<8 );
    Serial.print(Pitch_degs); Serial.print(", "); Serial.print(Roll_degs);
  }
  Serial.println();
}

void requestEvent()
{
  Wire.write("Hello Teensy"); // Send response to master
}

void setup()
{
  Serial.begin(9600); // keep the data transfers at low rate or the main loop gets upset

  // Initialise this external microcontroller I2C as a slave
  Wire.begin(slave_address);

  Wire.onReceive(receiveEvent);

  Wire.onRequest(requestEvent);

  Serial.println("ESP32 Slave Initialised");
}

void loop()
{
  // print or display your data values
}

*/