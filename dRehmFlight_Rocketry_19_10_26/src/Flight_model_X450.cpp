//Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3


  /* Spaceplane version: shamelessly over-complicated by Richard M. Newlands.

     Rockety bits added: vertical rocket ascents aiming only at the sky - "Automatic up-ness".
     To misqoute the late Rocketeer Harry Stine: "If you're an idiot or a Bad Guy, there's nothing for you here."
  */

   //========================================================================================================================//


// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Arduino.h>
#include <PWMServo.h>             // For servos
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands
#include "X450.h"                 // header file for X450 spaceplane flight model


//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

uint8_t flight_mode;
uint8_t PID_mode, previous_flight_mode;

float aileron_gain[NUM_MODES];
float B_loop_pitch[NUM_MODES], B_loop_roll[NUM_MODES];
float elevon_gain[NUM_MODES], Equivalent_upward_airspeed;
float hover_height_metres, hover_KD, hover_KI, hover_KP;
float i_limit[NUM_MODES];
float Last_altitude_metres;
float pitch_trim[NUM_MODES], port_elevon_trim[NUM_MODES];
float port_tilt_hover;
float Q_gain_reduction_factor, Q_gain_reduction_threshold;
float roll_yaw_mix_amount[NUM_MODES];
float servo_trim[NUM_SERVOS], servo_port_trim[NUM_MODES], servo_stbd_trim[NUM_MODES];
float stbd_elevon_trim[NUM_MODES];
float stbd_tilt_hover;
float tilt_trim[NUM_MODES];
float wing_motor_scaling;

Euler_angles PID, previous;

Euler_angles_degrees derivative;
Euler_angles_degrees error;
Euler_angles_degrees integral;// integral_inner_loop, integral_outer_loop;
Euler_angles_degrees previous_desired, previous_error;
Euler_angles_degrees previous_integral;// previous_integral_inner_loop, previous_integral_outer_loop;

PID_vals PIDs[NUM_MODES];

Fader fader[4];


/** Externals (yes, I know it's messy but it runs the loop quickly! RMN) **/

extern bool armed_to_fly;
extern bool Pid_low_throttle;
extern bool post_boost_unfreeze;
extern bool reallign_zero_pitch, rocket_boost;
extern bool tailsitting;

extern float Accelerometer_gees[3], Accelerometer_gees_prev[3], altitude_AMSL, Altitude_feet, Altitude_metres;
extern float Boost_unfreeze_time, Burn_time;
extern float delta_time, Density_ratio;
extern float height_metres;
extern float normalised_desired_throttle, normalised_emotor_command[NUM_EMOTORS], normalised_servo_command[NUM_SERVOS], normalised_pot[4];
extern float Rate_gyros_degs[3], Rate_gyros_degs_prev[3];
extern float serial_plot[5];

extern Euler_angles_degrees desired;
extern Euler_angles_degrees horizontal_IMU, previous_IMU, vertical_IMU;
extern Euler_angles_degrees input, input_rate;
extern Euler_angles normalised_desired;
extern Euler_angles passthru;

extern PID_vals mixed_PID;

extern PWMServo servo[NUM_SERVOS];

extern uint8_t IMU_accelerometer_range, IMU_rate_gyro_range;



//========================================================================================================================//


/* Attach model-specific servos */
void setup_servos()
{
  // Pin, min PWM value, max PWM value
  //for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
  //  servo[ubob].attach(servoPin[ubob], minUs_X450_SERVO, maxUs_X450_SERVO);

  servo[ONE].attach(servoPin[ONE], minUs_GH_S43D_SERVO, maxUs_GH_S43D_SERVO); // Port elevon
  servo[TWO].attach(servoPin[TWO], minUs_GH_S43D_SERVO, maxUs_GH_S43D_SERVO); // Stbd elevon

  servo[THREE].attach(servoPin[THREE], minUs_X450_TILT_SERVO, maxUs_X450_TILT_SERVO); // Port tilt servo
  servo[FOUR].attach(servoPin[FOUR],   minUs_X450_TILT_SERVO, maxUs_X450_TILT_SERVO); // Stbd tilt servo
}


/*** Setup controller parameters (take note of Nicholas' default values listed below before modifying!) ***/
void Setup_flight_controller()
{
  Altitude_metres = Last_altitude_metres = Equivalent_upward_airspeed = 0.0f;

  PID.roll = PID.pitch = PID.yaw = 0.0f;

  // Initialise four transition faders
  fader[ONE].initialise(FADE_UP);
  fader[TWO].initialise(FADE_DOWN);
  fader[THREE].initialise(FADE_UP);
  fader[FOUR].initialise(FADE_UP);


  port_tilt_hover = stbd_tilt_hover = 1.0f;

  Density_ratio = 1.0f;


  // Reduce servo gains with V squared (Dynamic pressure Q) above X metres per second
  Q_gain_reduction_threshold = 20.0f; // Exact value TBD (e.g. expected launcher exit velocity)

  if (Q_gain_reduction_threshold < 1.0f)
      Q_gain_reduction_threshold = 1.0f; // must be 1.0 or greater

  Q_gain_reduction_factor = 1.0f; // initialise


  // Select IMU gains
  IMU_accelerometer_range = ACCEL_2G; IMU_rate_gyro_range = GYRO_250DPS; 


  // Aircraft (and its IMU chip) is NOT initially on a vertical launchpad (tail-sitting)
  tailsitting         = false;
  reallign_zero_pitch = false;

  // Aircraft initially in horizontally-orientated hover mode
  previous_flight_mode = flight_mode = HOVER;

  // Desired hover height above ground
  hover_height_metres = 1.5f;

  
  /* Setup the rocketry */
  rocket_boost = true;

  Burn_time           = 6.3f; // rocketmotor/s burn time, seconds 
  Boost_unfreeze_time = 1.2f; // don't try to actively steer until motor/s have completed their boost thrust

  // Setup simulated rocketry flight computer
  Initialise_Rocketry();


  /* N.B: Derivative terms (Kd) that are too high amplify electrical noise at the emotors
     and can cause emotor overheating and burning out. Warm is okay, hot after 1 minute is too high!) */

  // Default Integral-term values, all aircraft and flight modes
  for (int ii = 0; ii < NUM_MODES; ii++)
    PIDs[ii].Ki.roll = PIDs[ii].Ki.pitch = 0.3f; // I-gain - angle mode and rate mode


  /** Hover mode, rocket motors post-burnout mass. **/

  if (PID_mode == CONTROL_ANGLE) // Angle hover
  {
    PIDs[HOVER].max.roll  =  30.0f; // max roll angle in degrees for angle mode,
    PIDs[HOVER].max.pitch =  30.0f; // max pitch angle in degrees for angle mode
    PIDs[HOVER].max.yaw   = 160.0f; // max yaw rate in degrees/sec

    // From F35 Anglehover code version
    PIDs[HOVER].Kp.roll =  0.55f; // Roll P-gain - angle mode
    PIDs[HOVER].Kd.roll = -0.1f;  // Roll D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_roll)  

    PIDs[HOVER].Kp.pitch =  0.55f; // Pitch P-gain - angle mode
    PIDs[HOVER].Kd.pitch = -0.1f;  // Pitch D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_pitch)

    /*
    PIDs[HOVER].Kp.roll =  0.2f;  // Roll P-gain - angle mode 
    PIDs[HOVER].Kd.roll = -0.05f; // Roll D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_roll)

    PIDs[HOVER].Kp.pitch =  0.2f;  // Pitch P-gain - angle mode
    PIDs[HOVER].Kd.pitch = -0.05f; // Pitch D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_pitch)
    */
  }
  else //if (PID_mode == CONTROL_RATE) // Rate hover
  {
    PIDs[HOVER].max.roll  = 180.0f; // max deg/sec for rate mode
    PIDs[HOVER].max.pitch = 180.0f; // max deg/sec for rate mode
    PIDs[HOVER].max.yaw   = 160.0f; // max yaw rate in degrees/sec

    PIDs[HOVER].Kp.roll = 0.3f;    // Roll P-gain - rate mode
    PIDs[HOVER].Kd.roll = 0.0209; //0.0001f; // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    PIDs[HOVER].Kp.pitch = 0.25f;   // Pitch P-gain - rate mode
    PIDs[HOVER].Kd.pitch = 0.0001f; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  }

  PIDs[HOVER].Kp.yaw = 0.42f;    // Yaw P-gain - rate mode
  PIDs[HOVER].Ki.yaw = 0.2f;     // Yaw I-gain - rate mode
  PIDs[HOVER].Kd.yaw = 0.00011f; // Yaw D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  

  // Servo trims - tune accordingly
  port_elevon_trim[HOVER] = 0.5f;
  stbd_elevon_trim[HOVER] = 0.5f;

  // Trim for tilt servo
  tilt_trim[HOVER] = 0.9f;

  // Vertical hold
  hover_KP = 0.5f; hover_KI = 0.0f; hover_KD = 0.1f;


  /** Forward flight **/
  if (PID_mode == CONTROL_ANGLE) // Angle hover
  {
    PIDs[FORWARD_FLIGHT].max.roll  = 440.0f; // max roll angle in degrees for angle mode, deg/sec for rate mode
    PIDs[FORWARD_FLIGHT].max.pitch = 440.0f; // max pitch angle in degrees for angle mode, deg/sec for rate mode
    PIDs[FORWARD_FLIGHT].max.yaw   = 200.0f; // max yaw rate in degrees/sec

    PIDs[FORWARD_FLIGHT].Kp.roll =  0.2f;  // Roll P-gain - angle mode 
    PIDs[FORWARD_FLIGHT].Kd.roll = -0.05f; // Roll D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_roll)

    PIDs[FORWARD_FLIGHT].Kp.pitch =  0.2f;  // Pitch P-gain - angle mode
    PIDs[FORWARD_FLIGHT].Kd.pitch= -0.05f; // Pitch D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_pitch)
  }
  else //if (PID_mode == CONTROL_RATE) // Rate hover
  {
    PIDs[FORWARD_FLIGHT].Kp.roll = 0.11f;   // Roll P-gain - rate mode
    PIDs[FORWARD_FLIGHT].Kd.roll = 0.0004f; // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    PIDs[FORWARD_FLIGHT].Kp.pitch = 0.17f;   // Pitch P-gain - rate mode
    PIDs[FORWARD_FLIGHT].Kd.pitch = 0.0004f; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  }

  PIDs[FORWARD_FLIGHT].Kp.yaw = 0.35f;    // Yaw P-gain - rate mode
  PIDs[FORWARD_FLIGHT].Ki.yaw = 0.05f;    // Yaw I-gain - rate mode
  PIDs[FORWARD_FLIGHT].Kd.yaw = 0.00005f; // Yaw D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

  // Servo trims - tune accordingly
  port_elevon_trim[FORWARD_FLIGHT] = 0.5f;
  stbd_elevon_trim[FORWARD_FLIGHT] = 0.5f;

  tilt_trim[FORWARD_FLIGHT] = 0.0f;


  /** ROCKET_ASCENT **/
  if (PID_mode == CONTROL_ANGLE) // Angle hover
  {
    PIDs[ROCKET_ASCENT].max.roll  = 440.0f; // max roll angle in degrees for angle mode, deg/sec for rate mode
    PIDs[ROCKET_ASCENT].max.pitch = 440.0f; // max pitch angle in degrees for angle mode, deg/sec for rate mode
    PIDs[ROCKET_ASCENT].max.yaw   = 200.0f; // max yaw rate in degrees/sec

    PIDs[ROCKET_ASCENT].Kp.roll =  0.2f;  // Roll P-gain - angle mode 
    PIDs[ROCKET_ASCENT].Kd.roll = -0.05f; // Roll D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_roll)

    PIDs[ROCKET_ASCENT].Kp.pitch =  0.2f;  // Pitch P-gain - angle mode
    PIDs[ROCKET_ASCENT].Kd.pitch= -0.05f; // Pitch D-gain - angle mode (if using controlANGLE2(), has no effect. Use B_loop_pitch)
  }
  else //if (PID_mode == CONTROL_RATE) // Rate hover
  {
    PIDs[ROCKET_ASCENT].max.roll  = 440.0f; // max roll angle in degrees for angle mode, deg/sec for rate mode
    PIDs[ROCKET_ASCENT].max.pitch = 440.0f; // max pitch angle in degrees for angle mode, deg/sec for rate mode
    PIDs[ROCKET_ASCENT].max.yaw   = 200.0f; // max yaw rate in degrees/sec

    PIDs[ROCKET_ASCENT].Kp.roll = 0.11f;   // Roll P-gain - rate mode
    PIDs[ROCKET_ASCENT].Kd.roll = 0.0004f; // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    PIDs[ROCKET_ASCENT].Kp.pitch = 0.17f;   // Pitch P-gain - rate mode
    PIDs[ROCKET_ASCENT].Kd.pitch = 0.0004f; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  }

  PIDs[ROCKET_ASCENT].Kp.yaw = 0.35f;    // Yaw P-gain - rate mode
  PIDs[ROCKET_ASCENT].Ki.yaw = 0.05f;    // Yaw I-gain - rate mode
  PIDs[ROCKET_ASCENT].Kd.yaw = 0.00005f; // Yaw D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

  // Servo trims - tune accordingly
  port_elevon_trim[ROCKET_ASCENT] = 0.5f;
  stbd_elevon_trim[ROCKET_ASCENT] = 0.5f;

  tilt_trim[ROCKET_ASCENT] = 0.0f;
}



/*** Run the flight controller ***/
void Run_flight_controller(uint32_t mode_mixer)
{
  float tail_fader;
  float yaw_torque;


  // Using Radio channel number 'MODE_MIX' for mixing between flight modes
  if (mode_mixer > 1666)
  {
    flight_mode  = HOVER;
    PID_mode     = CONTROL_ANGLE;
    rocket_boost = false;

    // Setup flight mode transitions
    if (previous_flight_mode == FORWARD_FLIGHT)
    {
      // For forward emotors and all servos
      fader[ONE].fade_time = 2.0f; // fade over 2 seconds
      fader[ONE].trigger(FADE_DOWN); // trigger the transition

      // For aft emotor fader
      fader[TWO].fade_time = 0.5f; // 0.5 seconds
      fader[TWO].trigger(FADE_UP); // trigger the transition
    }
  }
  else if (mode_mixer > 1333) // and less than 1666
  {
    flight_mode  = FORWARD_FLIGHT; // Horizontal flight
    PID_mode     = CONTROL_ANGLE;  // (See tailsitting video, but F35 was in rate mode)
    rocket_boost = false;

    // Setup flight mode transitions
    if (previous_flight_mode == HOVER)
    {
      // For forward emotors and all servos
      fader[ONE].fade_time = 2.0f; // fade over 2 seconds
      fader[ONE].trigger(FADE_UP); // trigger the transition
 
      // For aft emotor fader
      fader[TWO].fade_time = 0.5f; // 0.5 seconds
      fader[TWO].trigger(FADE_DOWN); // trigger the transition
    }
    else if (previous_flight_mode == ROCKET_ASCENT)
    {
      // In case of switch from angle mode to/from rate mode
      fader[THREE].fade_time = 0.5f; // 0.5 seconds
      fader[THREE].trigger(FADE_DOWN); // trigger the transition

      // For vertical to horizontal IMU reallignment
      fader[FOUR].fade_time = 2.0f; // 2 seconds
      fader[FOUR].trigger(FADE_DOWN); // trigger the transition
    }
  }
  else //if (mode_mixer < 1333)
  {
    flight_mode  = ROCKET_ASCENT; // Vertical ascent
    PID_mode     = CONTROL_RATE;  // Rate-mode stabilisation
    rocket_boost = true;

    // Setup flight mode transitions
    if (previous_flight_mode == FORWARD_FLIGHT)
    {
      // In case of switch from angle mode to/from rate mode
      fader[THREE].fade_time = 0.5f; // 0.5 seconds
      fader[THREE].trigger(FADE_UP); // trigger the transition

      // For horizontal to vertical IMU reallignment
      fader[FOUR].fade_time = 3.0f;
      fader[FOUR].trigger(FADE_UP); // trigger the transition
    }

    // Simulated rocketry computer (RDAS)
    // Rocketry_Computer(delta_time);
  }


  // Run the faders. Call ramp() only ONCE per loop
  fader[ONE].ramp(); // mixing using fader[ONE]
  fader[TWO].ramp(); // mixing using fader[TWO]
  fader[THREE].ramp(); // mixing using fader[THREE]
  fader[FOUR].ramp(); // mixing using fader[FOUR]


  /** Transition between different flight mode gains **/

  if (fader[ONE].transitioning == true) // HOVER to/from FORWARD_FLIGHT
  {
    mixed_PID.Kp.roll  = fader[ONE].Mix(PIDs[HOVER].Kp.roll, PIDs[FORWARD_FLIGHT].Kp.roll);
    mixed_PID.Kd.roll  = fader[ONE].Mix(PIDs[HOVER].Kd.roll, PIDs[FORWARD_FLIGHT].Kd.roll);

    mixed_PID.Kp.pitch = fader[ONE].Mix(PIDs[HOVER].Kp.pitch, PIDs[FORWARD_FLIGHT].Kp.pitch);
    mixed_PID.Kd.pitch = fader[ONE].Mix(PIDs[HOVER].Kd.pitch, PIDs[FORWARD_FLIGHT].Kd.pitch);

    mixed_PID.Kp.yaw   = fader[ONE].Mix(PIDs[HOVER].Kp.yaw, PIDs[FORWARD_FLIGHT].Kp.yaw);
    mixed_PID.Kd.yaw   = fader[ONE].Mix(PIDs[HOVER].Kd.yaw, PIDs[FORWARD_FLIGHT].Kd.yaw);

    mixed_PID.max.roll  = fader[ONE].Mix(PIDs[HOVER].max.roll,  PIDs[FORWARD_FLIGHT].max.roll);
    mixed_PID.max.pitch = fader[ONE].Mix(PIDs[HOVER].max.pitch, PIDs[FORWARD_FLIGHT].max.pitch);
    mixed_PID.max.yaw   = fader[ONE].Mix(PIDs[HOVER].max.yaw,   PIDs[FORWARD_FLIGHT].max.yaw);
  }
  else if (fader[THREE].transitioning == true) // In case of switch from angle mode to/from rate mode
  {
    mixed_PID.Kp.roll  = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kp.roll, PIDs[ROCKET_ASCENT].Kp.roll);
    mixed_PID.Kd.roll  = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kd.roll, PIDs[ROCKET_ASCENT].Kd.roll);

    mixed_PID.Kp.pitch = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kp.pitch, PIDs[ROCKET_ASCENT].Kp.pitch);
    mixed_PID.Kd.pitch = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kd.pitch, PIDs[ROCKET_ASCENT].Kd.pitch);

    mixed_PID.Kp.yaw   = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kp.yaw, PIDs[ROCKET_ASCENT].Kp.yaw);
    mixed_PID.Kd.yaw   = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].Kd.yaw, PIDs[ROCKET_ASCENT].Kd.yaw);

    mixed_PID.max.roll  = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].max.roll,  PIDs[ROCKET_ASCENT].max.roll);
    mixed_PID.max.pitch = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].max.pitch, PIDs[ROCKET_ASCENT].max.pitch);
    mixed_PID.max.yaw   = fader[THREE].Mix(PIDs[FORWARD_FLIGHT].max.yaw,   PIDs[ROCKET_ASCENT].max.yaw);
  }
  else // not transitioning
  {
    mixed_PID.Kp.roll  = PIDs[flight_mode].Kp.roll;
    mixed_PID.Kd.roll  = PIDs[flight_mode].Kd.roll;

    mixed_PID.Kp.pitch = PIDs[flight_mode].Kp.pitch;
    mixed_PID.Kd.pitch = PIDs[flight_mode].Kd.pitch;

    mixed_PID.Kp.yaw   = PIDs[flight_mode].Kp.yaw;
    mixed_PID.Kd.yaw   = PIDs[flight_mode].Kd.yaw;

    mixed_PID.max.roll  = PIDs[flight_mode].max.roll;
    mixed_PID.max.pitch = PIDs[flight_mode].max.pitch;
    mixed_PID.max.yaw   = PIDs[flight_mode].max.yaw;
  }

  
  //Serial.print(F(" mixed_PID.Kp.roll: ")); Serial.println(mixed_PID.Kp.roll);

  #if defined USING_ALTIMETER

    /* Spaceplanes and finned rockets can get squirrely at high airspeed! 
       Reduce servo gains with V squared (Dynamic pressure Q) above threshold airspeed.
       Based on A320 airliner joystick desensitiser code. */
    Equivalent_upward_airspeed = upward_velocity_metres_per_sec * sqrtf(Density_ratio);

    if (Equivalent_upward_airspeed > Q_gain_reduction_threshold)
      Q_gain_reduction_factor = sq(Q_gain_reduction_threshold / Equivalent_upward_airspeed);
    else
      Q_gain_reduction_factor = 1.0f;

  #else
    Q_gain_reduction_factor = 1.0f;
  #endif


  // Is the IMU chip horizontal or vertical?
  IMU_orientation(fader[FOUR], false);

  // Don't let integrators build if throttle is too low
  AttitudeController(PID_mode, Pid_low_throttle, false);


  /* 
  Mix scaled commands from PID controller to actuator outputs based on vehicle configuration.
  Takes PID.roll, PID.pitch, and PID.yaw computed from the PID controller and appropriately mixes them for the desired
  vehicle configuration.

  For example on a quadcopter, the left two motors should have +PID.roll while the right two motors
  should have -PID.roll. Front two should have -PID.pitch and the back two should have +PID.pitch etc... every motor has
  normalised (0 to 1) normalised_desired_throttle command for throttle control. Can also apply direct unstabilised
  commands from the transmitter with passthru.roll, passthru.pitch, and passthu.yaw

  Relevant variables:
  normalised_desired_throttle - direct thottle control
  PID.roll, PID.pitch, PID.yaw - stabilised axis variables
  passthru.roll, passthru.pitch, passthru.yaw - direct unstabilised command passthroughs

  Roll  = rotation about the X axis, +ve = starboard wing falling.   
  Pitch = rotation about the Y axis, +ve = nose falling.               
  Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight.
  */

  tail_fader = fader[TWO].mixer; // fade for 0.4 to 0.6 seconds for F35

  /** emotors: range is 0.0f to 1.0f **/
  normalised_emotor_command[PORT_ESC] = normalised_desired_throttle; 
  normalised_emotor_command[STBD_ESC] = normalised_desired_throttle;
  normalised_emotor_command[AFT_ESC]  = normalised_desired_throttle * tail_fader;


  /** servos: range is 0.0 to 1.0 with 0.5 being centred servo. **/
  normalised_servo_command[PORT_ELEVON] = port_elevon_trim[flight_mode];// + passthru.pitch - passthru.roll;
  normalised_servo_command[STBD_ELEVON] = stbd_elevon_trim[flight_mode];// + passthru.pitch + passthru.roll;


  if (flight_mode == HOVER)
  {
    /*
    // Add bank if yawing (yaw rate) for turn
    //float bank_with_yaw_rate = 0.508f - 0.034f * Rate_gyros_degs[HEADING_DECREASING];
    //float bank_with_yaw_rate = 0.035f * Rate_gyros_degs[HEADING_DECREASING];

    //normalised_emotor_command[STBD_ESC] -= bank_with_yaw_rate;
   */ 

    //normalised_emotor_command[PORT_ESC] -= PID.pitch + PID.roll;
    //normalised_emotor_command[STBD_ESC] -= PID.pitch - PID.roll; // - bank_with_yaw_rate;
    normalised_emotor_command[AFT_ESC]  += PID.pitch * tail_fader;
     
    // For debug
    normalised_emotor_command[PORT_ESC] = 0.0f;
    normalised_emotor_command[STBD_ESC] = 0.0f;
    //normalised_emotor_command[AFT_ESC]  = 0.0f;

    

    // Hover control PID 
    #if defined USING_ALTIMETER
    
      normalised_emotor_command[PORT_ESC] *= Hover_PID(delta_time);
      normalised_emotor_command[STBD_ESC] *= Hover_PID(delta_time);
      normalised_emotor_command[AFT_ESC]  *= Hover_PID(delta_time);
    
    #endif


    // Range 0.5 to 1.5
    tilt_trim[HOVER] = 0.9f; //2.0f * normalised_pot[ONE];

    //Serial.print(F(" tilt_trim[HOVER]:")); Serial.println(tilt_trim[HOVER]);

    yaw_torque = tilt_trim[HOVER] - PID.yaw;

    // Tilt servos predominantly vertically
    if (yaw_torque < 1.0f)
    {
      port_tilt_hover = 1.0f;
      stbd_tilt_hover = yaw_torque;
    }
    else
    {
      port_tilt_hover = 2.0f - yaw_torque;
      stbd_tilt_hover = 1.0f;
    }

    normalised_servo_command[PORT_TILT] = fader[ONE].Mix(port_tilt_hover, 0.0f);
    normalised_servo_command[STBD_TILT] = fader[ONE].Mix(stbd_tilt_hover, 0.0f);

  }
  else //if ((flight_mode == FORWARD_FLIGHT) || (flight_mode == ROCKET_ASCENT))
  { 
    /* emotors: range is 0.0f to 1.0f  
 
    normalised_emotor_command[PORT_ESC] += PID.yaw;
    normalised_emotor_command[STBD_ESC] -= PID.yaw;
    */

    normalised_servo_command[PORT_ELEVON] += (PID.roll + PID.pitch);
    normalised_servo_command[STBD_ELEVON] -= (PID.roll - PID.pitch);

    // Set tilt servos to horizontal
    normalised_servo_command[PORT_TILT] = fader[ONE].Mix(port_tilt_hover, 0.0f);
    normalised_servo_command[STBD_TILT] = fader[ONE].Mix(stbd_tilt_hover, 0.0f);


    //if ((flight_mode == ROCKET_ASCENT) && (rockets_firing))
    //emergency yaw to counteract one motor failing to light?

  }


  // Reduce elevon servos sensitivity for high-speed rocket ascents
  normalised_servo_command[PORT_ELEVON] *= Q_gain_reduction_factor;
  normalised_servo_command[STBD_ELEVON] *= Q_gain_reduction_factor;


  // Reverse the direction of the following servos:
  Reverse_servo(&normalised_servo_command[STBD_ELEVON]);
  Reverse_servo(&normalised_servo_command[PORT_TILT]);


  // Update previous value
  previous_flight_mode = flight_mode;
}





//========================================================================================================================//



void PrintFlightModes()
{
  if (flight_mode == HOVER)
    Serial.print(F(" Hover mode"));
  else if (flight_mode == FORWARD_FLIGHT)
    Serial.print(F(" Forward flight mode"));
  else if (flight_mode == ROCKET_ASCENT)
    Serial.print(F(" Rocket ascent mode"));

  Serial.print(F(" armed_to_fly:"));
  Serial.print(armed_to_fly);

  Serial.print(F(" Fader 1: "));
  Serial.print(fader[ONE].mixer);
  Serial.print(F(" Fader 2: "));
  Serial.print(fader[TWO].mixer);
  Serial.print(F(" Fader 3: "));
  Serial.print(fader[THREE].mixer);
  Serial.print(F(" Fader 4: "));
  Serial.print(fader[FOUR].mixer);

  Serial.println();
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

void PrintNormalisedServo()
{
  Serial.print(F("PORT_ELEVON:"));
  Serial.print(normalised_servo_command[PORT_ELEVON]);

  Serial.print(F(" STBD_ELEVON:"));
  Serial.print(normalised_servo_command[STBD_ELEVON]);

  Serial.print(F(" PORT_TILT:"));
  Serial.print(normalised_servo_command[PORT_TILT]);

  Serial.print(F(" STBD_TILT:"));
  Serial.print(normalised_servo_command[STBD_TILT]);

  Serial.println();
}