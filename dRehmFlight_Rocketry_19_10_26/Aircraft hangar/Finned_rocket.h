/* Header file for finned rocket flight model 
   Rick Newlands 3/6/26 */


#ifndef _FINNED_ROCKET_H_
#define _FINNED_ROCKET_H_

#include <Arduino.h>

/*** Use radio inputs? ***/
//#define USING_RADIO 


/*** Using electric motors? ***/
//#define USING_EMOTORS


/*** Using hobbyist rocket motors? ***/
#define USING_ROCKETS


/*** Using a BMPxxx altimeter for altitude estimation? ***/
#define USING_ALTIMETER 


// N.B. these can't be zero! Minimum number has to be 1
#define NUM_MODES    2 // number of modes in flight_mode_type
#define NUM_EMOTORS  1 // number of electric motors
#define NUM_SERVOS   4 // number of servos

// Number of fins/canards and IMU chip orientation - see controlMixer()
#define FOUR_FINS_HORIZONTAL_IMU


// Published values for bog-standard SG90 servo
#define minUs_SG90  544
#define maxUs_SG90 2400


enum flight_mode_type
{
  ROCKET_ASCENT,
  PARACHUTE_DESCENT
};


//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup


// PWM servo output pins:
const uint8_t servoPin[NUM_SERVOS] = {0,1,2,3};  //,4,5,6,7};


#endif /* _FINNED_ROCKET_H_ */