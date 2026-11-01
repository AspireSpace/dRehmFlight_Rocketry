/* Header file for A-frame rocket booster Flight model
   Rick Newlands 17/8/26 */


#ifndef _A_FRAME_BOOSTER_H_
#define _A_FRAME_BOOSTER_H_

#include <Arduino.h>

/*** Using radio control? ***/
#define USING_RADIO

/* sBus radio receiver type */
#define GRAUPNER_HOTT_GR12L_RX // For Graupner Hott GR12L sBus receiver

// Alter as required
#define NUM_CHANNELS 12 // number of radio channels (Up to 16 on an Sbus receiver, but my transmitter only uses 12)


/*** Using hobbyist rocket motors? ***/
#define USING_ROCKETS


// N.B. these can't be zero! Minimum number has to be 1
#define NUM_MODES    2 // number of modes in flight_mode_type
#define NUM_EMOTORS  1 // number of electric motors
#define NUM_SERVOS   6 // number of servos


// Published values for bog-standard SG90 servos
#define minUs_SG90  544
#define maxUs_SG90 2400

// Stated values for GH-S43D miniservos from PES
#define minUs_GH_S43D_SERVO 1000
#define maxUs_GH_S43D_SERVO 2000

// From Servocity database
#define minUs_HS_5245MG_SERVO  900
#define maxUs_HS_5245MG_SERVO 2100


enum flight_mode_type
{
  FORWARD_FLIGHT,
  ROCKET_ASCENT
};

enum servo_type
{
  PORT_ELEVON,
  STBD_ELEVON,
  PITCH_CANARD,
  TVC1,
  TVC2,
  TVC3
};


//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup


// PWM servo output pins:
const uint8_t servoPin[NUM_SERVOS] = {0,1,2,3};  //,4,5,6,7};


#endif /* _A_FRAME_BOOSTER_H_ */