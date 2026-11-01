//Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

/* This flight model: a finned HPR-class rocket vehicle with movable fins, canard foreplanes, or thrust-vectoring
 
   Rockety bits added: vertical rocket ascents aiming only at the sky - "Automatic up-ness".
   To misqoute the late Rocketeer Harry Stine: "If you're an idiot or a Bad Guy, there's nothing for you here."

   Initial version Rick N  23/9/25
   Recast Rick N  3/6/26
*/


// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Arduino.h>
#include <PWMServo.h>             // For servos
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands
#include "Finned_rocket.h"        // header file for finned rocket flight model



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

uint8_t flight_mode;
uint8_t PID_mode;

float cos_30, cos_60;
float Equivalent_upward_airspeed;
float i_limit[NUM_MODES];
float Last_altitude_metres;
float Kd.pitch[NUM_MODES], Kd.roll[NUM_MODES], Kd.yaw[NUM_MODES];
float Ki.pitch[NUM_MODES], Ki.roll[NUM_MODES], Ki.yaw[NUM_MODES];
float Kp.pitch[NUM_MODES], Kp.roll[NUM_MODES], Kp.yaw[NUM_MODES];
float max.roll[NUM_MODES], max.pitch[NUM_MODES], max.yaw[NUM_MODES];
float Q_gain_reduction_factor, Q_gain_reduction_threshold;
float servo_trim[NUM_SERVOS];

Euler_angles PID, previous;

Euler_angles_degrees derivative;
Euler_angles_degrees error;
Euler_angles_degrees input, input_rate, integral;
Euler_angles_degrees previous_desired, previous_error;
Euler_angles_degrees previous_integral;


/** Externals (yes, I know it's messy but it runs the loop quickly! RMN) **/

extern bool reallign_zero_pitch, rocket_boost;
extern bool tailsitting;
extern bool post_boost_unfreeze;

extern float altitude_AMSL, Altitude_feet, Altitude_metres;
extern float Boost_unfreeze_time, Burn_time;
extern float delta_time, Density_ratio;

extern float normalised_servo_command[NUM_SERVOS], normalised_pot[4];
extern float Rate_gyros_degs[3];
extern float upward_velocity_metres_per_sec;

extern Euler_angles_degrees desired;
extern Euler_angles_degrees horizontal_IMU, previous_IMU, vertical_IMU;

extern PWMServo servo[NUM_SERVOS];

extern uint8_t IMU_accelerometer_range, IMU_rate_gyro_range;


// Locals
void AttitudeController(uint8_t stabilisation_mode, bool safety_reset_PID_I);
void controlMixer(void);


//========================================================================================================================//


void setup_servos()
{
  // Pin, min PWM value, max PWM value
  for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
    servo[ubob].attach(servoPin[ubob], minUs_SG90, maxUs_SG90);
}



/*** Controller parameters (take note of Nicholas' default values listed below before modifying!) ***/
void Setup_flight_controller()
{
  Altitude_metres = Last_altitude_metres = Equivalent_upward_airspeed = 0.0f;

  PID.roll = PID.pitch = PID.yaw = 0.0f;

  Density_ratio = 1.0f;


  // Reduce servo gains with V squared (Dynamic pressure Q) above X metres per second
  Q_gain_reduction_threshold = 20.0f; // Exact value TBD (e.g. expected launcher exit velocity)

  if (Q_gain_reduction_threshold < 1.0f)
      Q_gain_reduction_threshold = 1.0f; // must be 1.0 or greater

  Q_gain_reduction_factor = 1.0f; // initialise


  // Larger g-range and faster rate required for rocketry
  IMU_accelerometer_range = ACCEL_16G; IMU_rate_gyro_range = GYRO_500DPS;

  // IMU initially in horizontal orientation?
  tailsitting         = false;
  reallign_zero_pitch = false;

  flight_mode = ROCKET_ASCENT;

  //PID_mode = CONTROL_ANGLE;
  PID_mode = CONTROL_RATE; // Rate-mode stabilisation


  /* Setup the rocketry */
  rocket_boost = true;

  Burn_time           = 6.3f; // rocketmotor/s burn time, seconds 
  Boost_unfreeze_time = 1.2f; // don't try to actively steer until motor/s have completed their boost thrust

  // Setup simulated rocketry flight computer
  Initialise_Rocketry();


  // Default Integral-term values
  Ki.roll[ROCKET_ASCENT] = Ki.pitch[ROCKET_ASCENT] = 0.3f; // I-gain - angle mode and rate mode


  if (PID_mode == CONTROL_ANGLE) // Angle mode
  {
    max.roll[ROCKET_ASCENT]  =  30.0f; // max roll angle in degrees for angle mode,
    max.pitch[ROCKET_ASCENT] =  30.0f; // max pitch angle in degrees for angle mode
    max.yaw[ROCKET_ASCENT]   = 160.0f; // max yaw rate in degrees/sec

    Kp.roll[ROCKET_ASCENT] = 0.55f; // Roll P-gain - angle mode
    Kd.roll[ROCKET_ASCENT] = -0.1f;  // Roll D-gain - angle mode

    Kp.pitch[ROCKET_ASCENT] = 0.55f; // Pitch P-gain - angle mode
    Kd.pitch[ROCKET_ASCENT] = -0.1f;  // Pitch D-gain - angle mode

  }
  else //if (PID_mode == CONTROL_RATE) // Rate mode
  {
    max.roll[ROCKET_ASCENT]  = 180.0f; // max deg/sec for rate mode
    max.pitch[ROCKET_ASCENT] = 180.0f; // max deg/sec for rate mode
    max.yaw[ROCKET_ASCENT]   = 160.0f; // max yaw rate in degrees/sec

    Kp.roll[ROCKET_ASCENT] = 0.3f;    // Roll P-gain - rate mode
    Kd.roll[ROCKET_ASCENT] = 0.02; //0.0001f; // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    Kp.pitch[ROCKET_ASCENT] = 0.25f;   // Pitch P-gain - rate mode
    Kd.pitch[ROCKET_ASCENT] = 0.0001f; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  }


  // Centralise the servo trims (N.B. normalised: nominally 0.5)
  servo_trim[ONE]   = 0.5f;
  servo_trim[TWO]   = 0.5f;
  servo_trim[THREE] = 0.5f;
  servo_trim[FOUR]  = 0.5f;

  // Trig used if using three fins
  cos_30 = 0.5f * sqrtf(3.0f);
  cos_60 = 0.5f;
}


/* Run the flight controller */
void Run_flight_controller(uint32_t mode_mixer)
{

  // Simulated rocketry computer (RDAS)
  Rocketry_Computer(delta_time);


  /* Spaceplanes and finned rockets can get squirrely at high airspeed! 
     Reduce servo gains with V squared (Dynamic pressure Q) above threshold airspeed.
     Based on A320 airliner joystick desensitiser code. */
  Equivalent_upward_airspeed = upward_velocity_metres_per_sec * sqrtf(Density_ratio);

  if (Equivalent_upward_airspeed > Q_gain_reduction_threshold)
    Q_gain_reduction_factor = sq(Q_gain_reduction_threshold / Equivalent_upward_airspeed);
  else
    Q_gain_reduction_factor = 1.0f;


  // (transition_fader_value) * (forward flight) + (1.0 - transition_fader_value) * (rocket ascent)
  Kp.roll[MIXER] = Kp.roll[flight_mode];
  Kd.roll[MIXER] = Kd.roll[flight_mode];

  Kp.pitch[MIXER] = Kp.pitch[flight_mode];
  Kd.pitch[MIXER] = Kd.pitch[flight_mode];

  Kp.yaw[MIXER] = Kp.yaw[flight_mode];
  Kd.yaw[MIXER] = Kd.yaw[flight_mode];

  max.roll[MIXER]  = max.roll[flight_mode];
  max.pitch[MIXER] = max.pitch[flight_mode];
  max.yaw[MIXER]   = max.yaw[flight_mode];





  /*
    Roll  = rotation about the X axis, +ve = starboard wing falling.   
    Pitch = rotation about the Y axis, +ve = nose falling.               
    Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight.
  */

  input.roll_degs  = horizontal_IMU.roll_degs;
  input.pitch_degs = horizontal_IMU.pitch_degs;
  input.yaw_degs   = horizontal_IMU.yaw_degs;

  input_rate.roll_degs  = Rate_gyros_degs[STBD_WING_DROPPING];
  input_rate.pitch_degs = Rate_gyros_degs[NOSE_DROPPING];
  input_rate.yaw_degs   = Rate_gyros_degs[HEADING_DECREASING];

  desired.roll_degs  = 0.0f;
  desired.pitch_degs = 0.0f;
  desired.yaw_degs   = 0.0f;


  AttitudeController(PID_mode, false, false);

  // Actuator mixing and scaling to PWM values
  controlMixer(); // Mixes PID outputs to normalised actuator commands
}



/* Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration.
   Takes PID.roll, PID.pitch, and PID.yaw computed from the PID controller and appropriately mixes them for the desired
   vehicle configuration.

   Relevant variables: PID.roll, PID.pitch, PID.yaw

   Roll  = rotation about the X axis, +ve = starboard wing falling.   
   Pitch = rotation about the Y axis, +ve = nose falling.               
   Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight. */
void controlMixer()
{
  // Three or four rocket fins or canards

  // TVC or active fin/foreplane servos
  normalised_servo_command[ONE]   = servo_trim[ONE];
  normalised_servo_command[TWO]   = servo_trim[TWO];
  normalised_servo_command[THREE] = servo_trim[THREE];
  normalised_servo_command[FOUR]  = servo_trim[FOUR];


  // Don't steer during the rocket/s boost thrust!
  if (post_boost_unfreeze)
  {
    #if defined THREE_FINS_HORIZONTAL_IMU

      // Resolve pitch rate and yaw rate PID demands into rotations about three rotation vectors 120 degrees apart (vehicle axes).
      normalised_servo_command[ONE]   += (PID.pitch * cos_30 - PID.roll * cos_60 + PID.yaw);
      normalised_servo_command[TWO]   += (PID.pitch * cos_30 + PID.roll * cos_60 + PID.yaw);
      normalised_servo_command[THREE] += (-PID.roll                              + PID.yaw);

    #elif defined THREE_FINS_VERTICAL_IMU

      // Resolve pitch rate and yaw rate PID demands into rotations about three rotation vectors 120 degrees apart (vehicle axes).
      normalised_servo_command[ONE]   += (PID.pitch * cos_30 - PID.yaw * cos_60 + PID.roll);
      normalised_servo_command[TWO]   += (PID.pitch * cos_30 + PID.yaw * cos_60 + PID.roll);
      normalised_servo_command[THREE] += (-PID.yaw                              + PID.roll);

    #elif defined FOUR_FINS_HORIZONTAL_IMU

      normalised_servo_command[ONE] += PID.pitch + PID.yaw; 
      normalised_servo_command[TWO] += PID.pitch - PID.yaw;

      normalised_servo_command[THREE] += PID.roll + PID.yaw;
      normalised_servo_command[FOUR]  += PID.roll - PID.yaw;

    #elif defined FOUR_FINS_VERTICAL_IMU

      normalised_servo_command[ONE] += PID.pitch + PID.roll;
      normalised_servo_command[TWO] += PID.pitch - PID.roll;

      normalised_servo_command[THREE] += PID.yaw + PID.roll;
      normalised_servo_command[FOUR]  += PID.yaw - PID.roll;

    #endif
  }


  // Reduce servo sensitivity for high-speed rocket ascents
  for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
    normalised_servo_command[ubob] *= Q_gain_reduction_factor;

}



//========================================================================================================================//


/* Basic PID control to stablise on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   can be thought of as 1-D stablised signals. They are mixed to the configuration of the vehicle in controlMixer(). */
void AttitudeController(uint8_t stabilisation_mode, bool safety_reset_PID_I, bool tune_the_PIDS)
{
  float i_limit = 25.0f; // Integrator saturation level, mostly for safety (default = 25.0)


  if (stabilisation_mode == CONTROL_ANGLE)
  {
    error.roll_degs  = desired.roll_degs  - input.roll_degs;
    error.pitch_degs = desired.pitch_degs - input.pitch_degs;

    derivative.roll_degs  = input_rate.roll_degs;
    derivative.pitch_degs = input_rate.pitch_degs;
  }
  else // if (stabilisation_mode == CONTROL_RATE)
  {
    // Same except the error is now the desired rate minus the rate gyro reading
    error.roll_degs  = desired.roll_degs  - input_rate.roll_degs;
    error.pitch_degs = desired.pitch_degs - input_rate.pitch_degs;

    derivative.roll_degs  = (error.roll_degs  - previous_error.roll_degs)  / delta_time;
    derivative.pitch_degs = (error.pitch_degs - previous_error.pitch_degs) / delta_time;
  }


  // Roll
  integral.roll_degs = previous_integral.roll_degs + error.roll_degs * delta_time;
  integral.roll_degs = constrain(integral.roll_degs, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup

  // Pitch
  integral.pitch_degs = previous_integral.pitch_degs + error.pitch_degs * delta_time;
  integral.pitch_degs = constrain(integral.pitch_degs, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup


  // Yaw is always stablised in rate mode
  error.yaw_degs = desired.yaw_degs - input_rate.yaw_degs;

  integral.yaw_degs = previous_integral.yaw_degs + error.yaw_degs * delta_time;
  integral.yaw_degs = constrain(integral.yaw_degs, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup

  derivative.yaw_degs = (error.yaw_degs - previous_error.yaw_degs) / delta_time;

  // Reset integrator - safety mode if throttle is low
  if (safety_reset_PID_I)
  {
    integral.pitch_degs = 0.0f;
    integral.roll_degs  = 0.0f;
    integral.yaw_degs   = 0.0f;
  }

  if (tune_the_PIDS)
  {
    /* Vary gains directly from your RC transmitter for PID tuneup. Comment-out once it's tuned-up!
       Increases the K gains by a factor of between 0.0 and 2.0 using rotary potentiometer inputs
       (if your transmitter has these). Rick N */

    error.roll_degs       *= 2.0f * normalised_pot[ONE]; // increase Kp.roll[flight_mode]
    derivative.roll_degs  *= 2.0f * normalised_pot[TWO]; // increase Kd.roll[flight_mode]

    error.pitch_degs      *= 2.0f * normalised_pot[THREE]; // increase Kp.pitch[flight_mode]
    derivative.pitch_degs *= 2.0f * normalised_pot[FOUR]; // increase Kd.pitch[flight_mode]
  }


  // PID commands
  PID.roll  = Kp.roll[MIXER]  * error.roll_degs  + Ki.roll[MIXER]  * integral.roll_degs  + Kd.roll[MIXER]  * derivative.roll_degs;
  PID.pitch = Kp.pitch[MIXER] * error.pitch_degs + Ki.pitch[MIXER] * integral.pitch_degs + Kd.pitch[MIXER] * derivative.pitch_degs;
  PID.yaw   = Kp.yaw[MIXER]   * error.yaw_degs   + Ki.yaw[MIXER]   * integral.yaw_degs   + Kd.yaw[MIXER]   * derivative.yaw_degs;


  // Scale by 0.01 to bring roughly within the range -1.0 to 1.0 
  PID.roll  *= 0.01f;
  PID.pitch *= 0.01f;
  PID.yaw   *= 0.01f;


  // Update roll variables
  previous_error.roll_degs    = error.roll_degs;
  previous_integral.roll_degs = integral.roll_degs;
  // Rate_gyros_degs_prev[X] = Rate_gyros_degs[X]; // bug! reuse of symbol

  // Update pitch variables
  previous_error.pitch_degs    = error.pitch_degs;
  previous_integral.pitch_degs = integral.pitch_degs;
  // Rate_gyros_degs_prev[Y] = Rate_gyros_degs[Y];  // bug! reuse of symbol

  // Update yaw variables
  previous_error.yaw_degs    = error.yaw_degs;
  previous_integral.yaw_degs = integral.yaw_degs;
}

void printPIDoutput()
{
  Serial.print(F("roll_PID:"));
  Serial.print(PID.roll);

  Serial.print(F(" pitch_PID:"));
  Serial.print(PID.pitch);

  Serial.print(F(" yaw_PID:"));
  Serial.println(PID.yaw);
}