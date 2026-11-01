/* Header file for X450 spaceplane flight model 
   Rick Newlands 3/6/26 */


#ifndef _X450_H_
#define _X450_H_

#include <Arduino.h>


/*** Using radio control? ***/
#define USING_RADIO

/* sBus radio receiver type */
//#define TARANIS_PLUS_RX      // For Taranis-Plus sBus receiver
//#define X4R_SB_RX            // For X4R-SB sBus receiver
#define GRAUPNER_HOTT_GR12L_RX // For Graupner Hott GR12L sBus receiver

// Alter as required
#define NUM_CHANNELS 12 // number of radio channels (Up to 16 on an Sbus receiver, but my transmitter only uses 12)


/*** Using electric motors? ***/
#define USING_EMOTORS


/*** Using hobbyist rocket motors? ***/
//#define USING_ROCKETS


/*** Using a BMPxxx altimeter for altitude estimation? ***/
//#define USING_ALTIMETER 


#define NUM_MODES    3 // number of modes in flight_mode_type
#define NUM_EMOTORS  3 // number of electric motors
#define NUM_SERVOS   4 // number of servos


// Measured values for X450 ESC servo inputs
#define minUs_X450_ESC 1000
#define maxUs_X450_ESC 2000

// Measured values for X450 tilt servos
#define minUs_X450_TILT_SERVO 1000
#define maxUs_X450_TILT_SERVO 2000

// Stated values for X450 elevon miniservos (oops I smoked them, replaced with GH-S43D's from PES)
#define minUs_GH_S43D_SERVO 1000
#define maxUs_GH_S43D_SERVO 2000


enum ESC_type // (alter to suit)
{
  PORT_ESC,
  STBD_ESC, // starboard
  AFT_ESC
};

enum servo_type // (alter to suit)
{
  PORT_ELEVON,
  STBD_ELEVON, // starboard
  PORT_TILT,
  STBD_TILT // starboard 
};

enum flight_mode_type
{
  HOVER,
  FORWARD_FLIGHT,
  ROCKET_ASCENT
};


//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup

// OneShot125 ESC pin output pins:
const uint8_t emotorPin[NUM_EMOTORS] = {4,5,6};  //,7};

// ESC servo-protocol output pins:
const uint8_t ESCservoPin[NUM_EMOTORS] = {4,5,6};  //,7};

// PWM servo output pins:
const uint8_t servoPin[NUM_SERVOS] = {0,1,2,3};


#endif /* _X450_H_ */