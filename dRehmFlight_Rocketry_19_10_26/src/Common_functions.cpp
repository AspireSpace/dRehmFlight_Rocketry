/* Useful mathematical functions

  This file contains useful maths routines

  Written by Rick Newlands et. al.
  */

// REQUIRED LIBRARIES
#include <Arduino.h>              // Required for Visual Studio Code and standard defines
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands
#include <math.h>



float serial_plot[5];


extern uint32_t current_time_microS;

extern float Altitude_metres; 
extern float delta_time;
extern float height_metres, hover_height_metres, hover_KD, hover_KI, hover_KP;
extern float Last_altitude_metres;
extern float normalised_pot[4];
extern float Rate_gyros_degs[3];
extern float upward_accel_metres_per_sec2, upward_velocity_metres_per_sec;

extern Euler_angles normalised_desired;
extern Euler_angles PID;

extern Euler_angles_degrees derivative, desired;
extern Euler_angles_degrees error;
extern Euler_angles_degrees horizontal_IMU;
extern Euler_angles_degrees input, input_rate, integral;
extern Euler_angles_degrees previous_error, previous_integral;
extern Euler_angles_degrees vertical_IMU;

extern PID_vals mixed_PID;


/* Low pass filter for Unsigned long integer variables (weighting must be less than or equal to 1.0)
   Writes the *values back into the 'current' and 'previous' values Rick N */
void low_pass_filter_UL(unsigned long *current, unsigned long *previous, float weighting)
{
  *current = (unsigned long) (weighting * *current + (1.0f - weighting) * *previous);

  // update last-times value
  *previous = *current;
}


/* Linear interpolation routine, with limiters added in case x isn't within the range range x1 to x2
   Limits updated to allow descending x values.
  x  = input value
  x1 = MINIMUM BOUNDS ( minimum x )
  y1 = MINIMUM VALUE  ( output value at MINIMUM BOUNDS )
  x2 = MAXIMUM BOUNDS ( maximum x )
  y2 = MAXIMUM VALUE  ( output value at MAXIMUM BOUNDS ) */
float LinearInterpolate(float x,
                        float x1, float y1, float x2, float y2)
{   
  // This procedure extrapolates the y value for the x position
  // on a line defined by x1,y1; x2,y2

  float c, m, y;  // the constants to find in y=mx+b

  if ( (x1 < x2) && ((x <= x1) || (x >= x2)) ) // ascending x values
  {
    if (x <= x1) return y1; else return y2;
  }
  else if ( (x1 > x2) && ((x >= x1) || (x <= x2)) ) // descending x values
  {
    if (x >= x1) return y1; else return y2;
  }
  else
  {
    m = (y2 - y1) / (x2 - x1); // calculate the gradient m
    c = y1 - m * x1;          // calculate the y-intercept c
    y = m * x + c;   // the final calculation
    return(y);
  }
}


/* DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time. 
   Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   and linearly interpolates that param variable between the maximum and minimum bounds. This function can be determined by
   monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being monitored to switch between
   two dynamic configurations (hover and forward flight), this function can be called within the logical statements in order 
   to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used to designate 
   the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
*/

/* Initialise the Transition fader. 
   Fader:: means 'this subroutine belongs to the class Fader' */
void Fader::initialise(float fade_direction)
{
  delta_param   = 0.0f;
  transitioning = false;

  if (fade_direction == FADE_UP)
    mixer = 0.0f;
  else // fade down
    mixer = 1.0f;
}

/* Trigger the Transition fader */
void Fader::trigger(float fade_direction)
{
  //start_microS = current_time_microS; // microseconds

  transitioning = true;

  // Difference in seconds (+ve or -ve) to add or subtract from param for each loop iteration for desired fade_time
  delta_param = fade_direction / fade_time;

  if (fade_direction == FADE_UP)
    mixer = 0.0f;
  else // fade down
    mixer = 1.0f;
}

/* Linearly ramp the Transition fader */
void Fader::ramp(void)
{
  // Increase param by delta_param for each loop iteration
  mixer += delta_param * delta_time;

  // Constrain the value
  if ((delta_param > 0.0f) && (mixer > 1.0f))
  {
    delta_param   = 0.0f;
    transitioning = false; // clear this flag
  }
  else if ((delta_param < 0.0f) && (mixer < 0.0f))
  {
    delta_param   = 0.0f;
    transitioning = false; // clear this flag
  }
}

/* Linearly mix between two values using the Transition fader */
float Fader::Mix(float param1, float param2)
{
  // Linearly mix between two values
  return( LinearInterpolate(mixer, 0.0f, param1, 1.0f, param2) );
}


/* Reverse the direction of a servo.
 '*' means write back into the servo variable.
   Rick N June 2026 */
void Reverse_servo(float *normalised_servo_value)
{
  *normalised_servo_value = 1.0f - *normalised_servo_value;
}

/* Facilitate tailsitting and vertical ascents */
void IMU_orientation(Fader reorientate, bool drone_mode)
{
  /*
    Roll  = rotation about the X axis, +ve = starboard wing falling.   
    Pitch = rotation about the Y axis, +ve = nose falling.               
    Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight.
  */


  /* Transition between horizontal and vertical IMU orientations to avoid gimbal lock.
     TBD this might be better handled by a pitch transition from last aircraft climb angle to vertical
     drone mode at 90 degrees of pitch. */
  input.roll_degs  = reorientate.Mix(horizontal_IMU.roll_degs,  vertical_IMU.roll_degs);
  input.pitch_degs = reorientate.Mix(horizontal_IMU.pitch_degs, vertical_IMU.pitch_degs);
  input.yaw_degs   = reorientate.Mix(horizontal_IMU.yaw_degs,   vertical_IMU.yaw_degs);


  /* Rate gyros */
  input_rate.pitch_degs = Rate_gyros_degs[NOSE_DROPPING];

  if (drone_mode) // 'Tailsitting' mode: you might want to swap X and Z rate gyros during vertical ascents
  {
    input_rate.roll_degs = reorientate.Mix(Rate_gyros_degs[STBD_WING_DROPPING], Rate_gyros_degs[HEADING_DECREASING]);
    input_rate.yaw_degs  = reorientate.Mix(Rate_gyros_degs[HEADING_DECREASING], Rate_gyros_degs[STBD_WING_DROPPING]);
  }
  else // don't swap
  {
    input_rate.roll_degs = Rate_gyros_degs[STBD_WING_DROPPING]; // X
    input_rate.yaw_degs  = Rate_gyros_degs[HEADING_DECREASING]; // Z
  }


  // Desired.roll_degs and desired.pitch_degs are scaled to be within max roll, max pitch amount in either degrees (angle mode)
  // or degrees/sec (rate mode).  desired.yaw_degs is scaled to be within max yaw in degrees/sec.
  desired.roll_degs  = normalised_desired.roll  * mixed_PID.max.roll;  // Between -max.roll  and +max.roll
  desired.pitch_degs = normalised_desired.pitch * mixed_PID.max.pitch; // Between -max.pitch and +max.pitch
  desired.yaw_degs   = normalised_desired.yaw   * mixed_PID.max.yaw;   // Between -max.yaw   and +max.yaw
}


/*
  Basic PID control to stablise on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
  getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
  are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
  excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
  they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
  if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
  terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
  can be thought of as 1-D stablised signals. They are mixed to the configuration of the vehicle in controlMixer().
*/
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

  if (true)//tune_the_PIDS)
  {
    /* Vary gains directly from your RC transmitter for PID tuneup. Comment-out once it's tuned-up!
       Increases the K gains by a factor of between 0.0 and 2.0 using rotary potentiometer inputs
       (if your transmitter has these). Rick N */

    //mixed_PID.Kp.pitch *= 2.0f * normalised_pot[ONE];
    //mixed_PID.Kd.pitch *= 2.0f * normalised_pot[TWO];
    //mixed_PID.Ki.pitch *= 2.0f * normalised_pot[THREE];

    //mixed_PID.Kd.pitch = 0.0f;
    //mixed_PID.Ki.pitch = 0.0f;

    serial_plot[ONE]   = mixed_PID.Kp.pitch * error.pitch_degs      * 0.01f;
    serial_plot[TWO]   = mixed_PID.Ki.pitch * integral.pitch_degs   * 0.01f;
    serial_plot[THREE] = mixed_PID.Kd.pitch * derivative.pitch_degs * 0.01f;

    // Set graph Y-axis range
    serial_plot[FOUR] = -0.2f;
    serial_plot[FIVE] =  0.2f;
  }


  // PID commands
  PID.roll  = mixed_PID.Kp.roll  * error.roll_degs  + mixed_PID.Ki.roll  * integral.roll_degs  + mixed_PID.Kd.roll  * derivative.roll_degs;
  PID.pitch = mixed_PID.Kp.pitch * error.pitch_degs + mixed_PID.Ki.pitch * integral.pitch_degs + mixed_PID.Kd.pitch * derivative.pitch_degs;
  PID.yaw   = mixed_PID.Kp.yaw   * error.yaw_degs   + mixed_PID.Ki.yaw   * integral.yaw_degs   + mixed_PID.Kd.yaw   * derivative.yaw_degs;

  // Scale by 0.01 to bring roughly within the range -1.0 to 1.0 
  PID.roll  *= 0.01f;
  PID.pitch *= 0.01f;
  PID.yaw   *= 0.01f;


  // Update roll variables
  previous_error.roll_degs    = error.roll_degs;
  previous_integral.roll_degs = integral.roll_degs;

  // Update pitch variables
  previous_error.pitch_degs    = error.pitch_degs;
  previous_integral.pitch_degs = integral.pitch_degs;

  // Update yaw variables
  previous_error.yaw_degs    = error.yaw_degs;
  previous_integral.yaw_degs = integral.yaw_degs;
}


/* Hover mode vertical rate-of-climb PID 
   Rick N May 2026 TBD */
float Hover_PID(float d_time)
{ 
  float correction;
  float derivative_term;
  float integral_term;
  float proportional_term;
  float velocity_from_altimeter;
   
  // metres per second
  velocity_from_altimeter = (Altitude_metres - Last_altitude_metres) / d_time;

  //velocity_from_altimeter = constrain(velocity_from_altimeter, -3.0, 3.0); // +/- 3 m/s

  // Save last-times value
  Last_altitude_metres = Altitude_metres;


  // Complementary filter added to the vertical velocity (from Multiwii)
  proportional_term = upward_velocity_metres_per_sec * 0.985f + velocity_from_altimeter * 0.015f;

  integral_term = hover_height_metres - height_metres;

  // Always constrain integral terms, or they'll explode!
  integral_term = constrain(integral_term, -0.5f, 0.5f);

  derivative_term = upward_accel_metres_per_sec2;

  correction = hover_KP * proportional_term + hover_KI * integral_term + hover_KD * derivative_term;

  return(correction);
}


/* Mario Zechner's 'Serial-plotter'. A visual studio version of the Arduino serial plotter.
   https://marketplace.visualstudio.com/items?itemName=badlogicgames.serial-plotter 
   In VS Code, press CTRL + SHIFT + P (CMD + SHIFT + P on macOS) to open the command palette.
   Type Serial Plotter: Open pane and select the command. */
void Serial_plot(void)
{
  Serial.print(">"); // Trigger-symbol to send subsequent data to 'Serial-plotter'

  Serial.print("Plot1:");
  Serial.print(serial_plot[ONE]);
  Serial.print(",");

  Serial.print("Plot2:");
  Serial.print(serial_plot[TWO]);
  Serial.print(",");

  Serial.print("Plot3:");
  Serial.print(serial_plot[THREE]);
  Serial.print(",");

  Serial.print("Plot4:");
  Serial.print(serial_plot[FOUR]);
  Serial.print(",");

  Serial.print("Plot5:");
  Serial.print(serial_plot[FIVE]);

  Serial.println(); // Writes \r\n symbols to stop writing data to Mario Zechner's 'Serial-plotter'
}