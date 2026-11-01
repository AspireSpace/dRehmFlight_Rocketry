//Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

/* This model: a tailsitting Spaceplane

   N.B. a stabilised purely-vertical ascent only,
   i.e. 'Automatic up-ness'
 
   Rockety bits added: vertical rocket ascents aiming only at the sky - "Automatic up-ness".
   To misqoute the late Rocketeer Harry Stine: "If you're an idiot or a Bad Guy, there's nothing for you here."

   Initial version Rick N  23/9/25
   Recast Rick N  3/6/26
*/


// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Arduino.h>
#include <PWMServo.h>             // For servos
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands
#include "Tailsitter.h"           // header file for A-frame rocket booster flight model



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

uint8_t flight_mode;
uint8_t PID_mode;

float canard_trim[NUM_MODES], cos_30, cos_60;
float Equivalent_upward_airspeed;
float i_limit[NUM_MODES];
float Last_altitude_metres;
float Kd.pitch[NUM_MODES], Kd.roll[NUM_MODES], Kd.yaw[NUM_MODES];
float Ki.pitch[NUM_MODES], Ki.roll[NUM_MODES], Ki.yaw[NUM_MODES];
float Kp.pitch[NUM_MODES], Kp.roll[NUM_MODES], Kp.yaw[NUM_MODES];
float max.roll[NUM_MODES], max.pitch[NUM_MODES], max.yaw[NUM_MODES];
float port_elevon_trim[NUM_MODES];
float Q_gain_reduction_factor, Q_gain_reduction_threshold;
float servo_trim[NUM_SERVOS], stbd_elevon_trim[NUM_MODES];
float transition_fader_value;

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
extern bool fade_up;

extern float altitude_AMSL, Altitude_feet, Altitude_metres;
extern float Boost_unfreeze_time, Burn_time;
extern float delta_time, Density_ratio;

extern float normalised_servo_command[NUM_SERVOS], normalised_pot[4];
extern float Rate_gyros_degs[3];
extern float upward_velocity_metres_per_sec;

extern Euler_angles_degrees desired;
extern Euler_angles_degrees horizontal_IMU, previous_IMU, vertical_IMU;

extern Euler_angles normalised_desired;
extern Euler_angles passthru;

extern PWMServo servo[NUM_SERVOS];

extern uint8_t IMU_accelerometer_range, IMU_rate_gyro_range;


// Locals
void AttitudeController(uint8_t stabilisation_mode, bool safety_reset_PID_I, bool tune_the_PIDS);
void IMU_orientation(bool drone_mode);
void controlMixer(void);
float mode_mixing(float mixer, float mode1, float mode2);
float NormalisedLinearFader(float param, float fadeTime, bool state);
void Reverse_servo(float *normalised_servo_value);


//========================================================================================================================//


void setup_servos()
{
  // Pin, min PWM value, max PWM value
  servo[PORT_ELEVON].attach(servoPin[ONE], minUs_GH_S43D_SERVO, maxUs_GH_S43D_SERVO); // Port elevon or V-tail
  servo[STBD_ELEVON].attach(servoPin[TWO], minUs_GH_S43D_SERVO, maxUs_GH_S43D_SERVO); // Stbd elevon or V-tail

  servo[PITCH_CANARD].attach(servoPin[THREE], minUs_HS_5245MG_SERVO, maxUs_HS_5245MG_SERVO); // Canard/pitch trim servo

  /*
  servo[TVC1].attach(servoPin[FOUR], minUs_SG90, maxUs_SG90);
  servo[TVC2].attach(servoPin[FIVE], minUs_SG90, maxUs_SG90);
  servo[TVC3].attach(servoPin[SIX],  minUs_SG90, maxUs_SG90);
  */
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

  // IMU initially in vertical orientation
  tailsitting         = true;
  reallign_zero_pitch = true;

  flight_mode = ROCKET_ASCENT;

  //PID_mode = CONTROL_ANGLE;
  PID_mode = CONTROL_RATE; // Rate-mode stabilisation
  fade_up         = true;  // fade upwards


  /* Setup the rocketry */
  rocket_boost = true;

  Burn_time           = 6.3f; // rocketmotor/s burn time, seconds 
  Boost_unfreeze_time = 1.2f; // don't try to actively steer until motor/s have completed their boost thrust

  // Setup simulated rocketry flight computer
  Initialise_Rocketry();


  // Default Integral-term values - angle mode and rate mode
  Ki.roll[ROCKET_ASCENT] = Ki.pitch[ROCKET_ASCENT] = Ki.roll[FORWARD_FLIGHT] = Ki.pitch[FORWARD_FLIGHT] = 0.3f;

  transition_fader_value = 0.0f; // 1.0f is forward flight and horizontal IMU, 0.0f is ROCKET_ASCENT and vertical IMU


  if (PID_mode == CONTROL_ANGLE) // Angle mode
  {
    max.roll[ROCKET_ASCENT]  =  30.0f; // max roll angle in degrees for angle mode,
    max.pitch[ROCKET_ASCENT] =  30.0f; // max pitch angle in degrees for angle mode
    max.yaw[ROCKET_ASCENT]   = 160.0f; // max yaw rate in degrees/sec

    Kp.roll[ROCKET_ASCENT] = 0.55f; // Roll P-gain - angle mode
    Kd.roll[ROCKET_ASCENT] = -0.1f;  // Roll D-gain - angle mode

    Kp.pitch[ROCKET_ASCENT] = 0.55f; // Pitch P-gain - angle mode
    Kd.pitch[ROCKET_ASCENT] = -0.1f;  // Pitch D-gain - angle mode


    // Forward flight mode
    max.roll[FORWARD_FLIGHT]  = 50.0f; // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
    max.pitch[FORWARD_FLIGHT] = 50.0f; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
    max.yaw[FORWARD_FLIGHT]  = 220.0f; // Max yaw rate in deg/sec

    Kp.roll[FORWARD_FLIGHT] = 0.5f;  // Roll P-gain - angle mode
    Kd.roll[FORWARD_FLIGHT] = -0.06f; // Roll D-gain - angle mode

    Kp.pitch[FORWARD_FLIGHT] = 0.25f; // Pitch P-gain - angle mode
    Kd.pitch[FORWARD_FLIGHT] = -0.05f; // Pitch D-gain - angle mode

    Kp.yaw[FORWARD_FLIGHT] = 0.2f;     // Yaw P-gain - rate mode
    Kd.yaw[FORWARD_FLIGHT] = 0.00017f; // Yaw D-gain - rate mode

  }
  else //if (PID_mode == CONTROL_RATE) // Rate mode
  {
    max.roll[ROCKET_ASCENT]  = 180.0f; // max deg/sec for rate mode
    max.pitch[ROCKET_ASCENT] = 180.0f; // max deg/sec for rate mode
    max.yaw[ROCKET_ASCENT]   = 160.0f; // max yaw rate in degrees/sec

    Kp.roll[ROCKET_ASCENT] = 0.3f; // Roll P-gain - rate mode
    Kd.roll[ROCKET_ASCENT] = 0.02; //0.0001f; // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

    Kp.pitch[ROCKET_ASCENT] = 0.25f;   // Pitch P-gain - rate mode
    Kd.pitch[ROCKET_ASCENT] = 0.0001f; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  }

  Kp.yaw[ROCKET_ASCENT] = 0.2f;     // Yaw P-gain - rate mode only
  Kd.yaw[ROCKET_ASCENT] = 0.00017f; // Yaw D-gain - rate mode only



  // servo trims (N.B. normalised: nominally 0.5)
  port_elevon_trim[FORWARD_FLIGHT] = 0.5f;
  stbd_elevon_trim[FORWARD_FLIGHT] = 0.5f;
  canard_trim[FORWARD_FLIGHT]      = 0.5f;

  port_elevon_trim[ROCKET_ASCENT] = 0.5f;
  stbd_elevon_trim[ROCKET_ASCENT] = 0.5f;
  canard_trim[ROCKET_ASCENT]      = 0.5f;

  //pitch_trim[ROCKET_ASCENT] = 0.0f; // pitch trim angle in degrees to command in forward/gliding flight

  servo_trim[TVC1] = 0.5f;
  servo_trim[TVC2] = 0.5f;
  servo_trim[TVC3] = 0.5f;


  // Trig used for the three TVC servos
  cos_30 = 0.5f * sqrtf(3.0f);
  cos_60 = 0.5f;
}


/* Run the flight controller */
void Run_flight_controller(uint32_t mode_mixer)
{

  // Simulated rocketry computer (RDAS)
  Rocketry_Computer(delta_time);

  
  // Using Radio channel number 'MODE_MIX' for mixing between flight modes
  if (mode_mixer > 1500)
  {
    flight_mode  = FORWARD_FLIGHT; // Horizontal glide
    PID_mode     = CONTROL_ANGLE;
    rocket_boost = false;
    fade_up      = true; // fade upwards towards 1.0f
  }
  else
  {
    flight_mode  = ROCKET_ASCENT; // Vertical ascent
    PID_mode     = CONTROL_RATE; // Rate-mode stabilisation
    rocket_boost = true;
    fade_up      = false; // fade downwards towards 0.0f

    // Simulated rocketry computer (RDAS)
    //Rocketry_Computer(delta_time);
  }


  /* Mode transitions between different gains */

  // Slew over X seconds
  transition_fader_value = NormalisedLinearFader(transition_fader_value, 2.0f, fade_up);
  //Serial.println(transition_fader_value);

  // (transition_fader_value) * (forward flight) + (1.0 - transition_fader_value) * (rocket ascent)
  Kp.roll[MIXER]  = mode_mixing(transition_fader_value, Kp.roll[FORWARD_FLIGHT],  Kp.roll[ROCKET_ASCENT]);
  Kd.roll[MIXER]  = mode_mixing(transition_fader_value, Kd.roll[FORWARD_FLIGHT],  Kd.roll[ROCKET_ASCENT]);

  Kp.pitch[MIXER] = mode_mixing(transition_fader_value, Kp.pitch[FORWARD_FLIGHT], Kp.pitch[ROCKET_ASCENT]);
  Kd.pitch[MIXER] = mode_mixing(transition_fader_value, Kd.pitch[FORWARD_FLIGHT], Kd.pitch[ROCKET_ASCENT]);

  Kp.yaw[MIXER]   = mode_mixing(transition_fader_value, Kp.yaw[FORWARD_FLIGHT],   Kp.yaw[ROCKET_ASCENT]);
  Kd.yaw[MIXER]   = mode_mixing(transition_fader_value, Kd.yaw[FORWARD_FLIGHT],   Kd.yaw[ROCKET_ASCENT]);

  max.roll[MIXER]  = mode_mixing(transition_fader_value, max.roll[FORWARD_FLIGHT],  max.roll[ROCKET_ASCENT]);
  max.pitch[MIXER] = mode_mixing(transition_fader_value, max.pitch[FORWARD_FLIGHT], max.pitch[ROCKET_ASCENT]);
  max.yaw[MIXER]   = mode_mixing(transition_fader_value, max.yaw[FORWARD_FLIGHT],   max.yaw[ROCKET_ASCENT]);


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



  // Is the IMU chip horizontal or vertical? and do you want drone mode for vertical ascents?
  IMU_orientation(fader[TWO], false); 

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

  /** servos: range is 0.0f to 1.0f with 0.5f being centred servo. **/

  port_elevon_trim[MIXER] = mode_mixing(transition_fader_value, port_elevon_trim[FORWARD_FLIGHT], port_elevon_trim[ROCKET_ASCENT]);
  stbd_elevon_trim[MIXER] = mode_mixing(transition_fader_value, stbd_elevon_trim[FORWARD_FLIGHT], stbd_elevon_trim[ROCKET_ASCENT]);

  // Swift V-tail servos or Shuttle elevons
  normalised_servo_command[PORT_ELEVON] = port_elevon_trim[MIXER] + passthru.pitch; // Port elevon
  normalised_servo_command[STBD_ELEVON] = stbd_elevon_trim[MIXER] + passthru.pitch; // Starboard elevon (negated)
  /*
  normalised_servo_command[PORT_ELEVON] = (PID.roll + PID.pitch) + port_elevon_trim[MIXER]; // Port elevon
  normalised_servo_command[STBD_ELEVON] = (PID.roll - PID.pitch) + stbd_elevon_trim[MIXER]; // Starboard elevon (negated)
  */

  // Gains
  normalised_servo_command[PORT_ELEVON] *= 0.1f;
  normalised_servo_command[STBD_ELEVON] *= 0.1f;

  // TVC servos
  normalised_servo_command[TVC1] = servo_trim[TVC1];
  normalised_servo_command[TVC2] = servo_trim[TVC2];
  normalised_servo_command[TVC3] = servo_trim[TVC3];

  
  if (flight_mode == ROCKET_ASCENT) // && (rockets_firing))
  {
    /*
    port_elevon_trim[HOVER] = normalised_pot[ONE];
    stbd_elevon_trim[HOVER] = normalised_pot[TWO];

    Serial.print(F(" port_elevon_trim[HOVER]:"));
    Serial.print(port_elevon_trim[HOVER]);

    Serial.print(F(" stbd_elevon_trim[HOVER]:"));
    Serial.println(stbd_elevon_trim[HOVER]);
    */

    // Pitch canard
    normalised_servo_command[PITCH_CANARD] = canard_trim[ROCKET_ASCENT] + passthru.pitch * 0.1f; // times gain
    //normalised_servo_command[PITCH_CANARD] = canard_trim[ROCKET_ASCENT] - PID.pitch * 0.1f; // times gain


    // Don't steer during the rocket/s boost thrust!
    if (post_boost_unfreeze)
    {

      /* THREE_FINS_VERTICAL_IMU */

      // Resolve pitch rate and yaw rate PID demands into rotations about three rotation vectors 120 degrees apart (vehicle axes).
      normalised_servo_command[TVC1] += (PID.pitch * cos_30 - PID.yaw * cos_60 + PID.roll);
      normalised_servo_command[TVC2] += (PID.pitch * cos_30 + PID.yaw * cos_60 + PID.roll);
      normalised_servo_command[TVC3] += (-PID.yaw                              + PID.roll);

      // Gains
      normalised_servo_command[TVC1] *= 0.1f;
      normalised_servo_command[TVC2] *= 0.1f;
      normalised_servo_command[TVC3] *= 0.1f;
    }

    #if defined USING_ALTIMETER

      // Reduce servo sensitivity for high-speed rocket ascents
      for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
        normalised_servo_command[ubob] *= Q_gain_reduction_factor;

    #endif   
  }

  // Reverse the direction of servos
  //Reverse_servo(&normalised_servo_command[STBD_ELEVON]);
  //Reverse_servo(&normalised_servo_command[TVC]);

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




void PrintFlightModes()
{
  if (flight_mode == FORWARD_FLIGHT)
    Serial.print(F(" Forward flight mode"));
  else if (flight_mode == ROCKET_ASCENT)
    Serial.print(F(" Rocket ascent mode"));

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
  Serial.print(F("normalised1:"));
  Serial.print(normalised_servo_command[ONE]);

  Serial.print(F(" normalised2:"));
  Serial.print(normalised_servo_command[TWO]);

  Serial.print(F(" normalised3:"));
  Serial.print(normalised_servo_command[THREE]);

  Serial.print(F(" normalised4:"));
  Serial.print(normalised_servo_command[FOUR]);

  /*
  Serial.print(F(" normalised5:"));
  Serial.print(normalised_servo_command[FIVE]));

  Serial.print(F(" normalised6:"));
  Serial.print(normalised_servo_command[SIX]);
  */

  Serial.println();
}