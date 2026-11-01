/* Teensy Flight Controller - dRehmFlight
   Author: Nicholas Rehm
   Last Updated: 7/29/2022  Beta 1.3

   Spaceplane version: shamelessly over-complicated by Richard M. Newlands.
   Rockety bits added: vertical rocket ascents aiming only at the sky: "Automatic up-ness".
   To misqoute the late Rocketeer Harry Stine: "If you're an idiot or a Bad Guy, there's nothing for you here."

//========================================================================================================================//

// CREDITS + SPECIAL THANKS
 
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

Thanks to: the United Kingdom Rocketry Association, Aspirespace, and UKSEDS, for not running away screaming - Rick N
*/

//========================================================================================================================//


// REQUIRED LIBRARIES
#include <Arduino.h>       // Required for Visual Studio Code and standard defines
#include <Wire.h>           // I2C communication
#include <PWMServo.h>         // Commanding any extra actuators
#include "SBUS.h"                // sBus interface for radio receiver
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands

// Choose flight model from subfolder 'aircraft hangar'
//#include "Aframe_booster.h" // Header file for A-frame rocket booster Flight model
#include "X450.h" // header file for X450 winged drone flight model
//#include "Finned_rocket.h" // header file for finned rocket flight model



/* Setup SBUS radio receiver on Teensy serial port 5 (pin 21 - RX5) */
SBUS sbus(Serial5);


/* Print to the serial monitor for debugging */
#define PRINT_FOR_DEBUG


/*** Comment out just for debugging ***/
#define USING_SERVOS


/* If using electric motors, use either the servo protocol or the 125 protocol for the ESC's, but not both! */
#if defined USING_EMOTORS
  #define ESC_SERVO_PROTOCOL
  //#define ESC_125_PROTOCOL

  // Create servo objects to control emotor ESC's using standard servo PWM output
  #if defined ESC_SERVO_PROTOCOL
    PWMServo ESCservo[NUM_EMOTORS];
  #endif
#endif


/*** Create servo objects to control servos using standard servo PWM output ***/
#if defined USING_SERVOS
  PWMServo servo[NUM_SERVOS];
#endif



// DECLARE GLOBAL VARIABLES

/*** Desired loop frequency ***/
const unsigned int loopHz = 2000; // Loop frequency in Hz. Do not exceed 2000 Hz, all filter parameters tuned to 2000 Hz by default

bool armed_to_fly;
bool blinkAlternate;
bool fade_up;
bool Pid_low_throttle;
bool reallign_zero_pitch, rocket_boost;
bool sbusFailSafe, sbusLostFrame;
bool tailsitting;
bool warmed_up;

// Rick N cannot be uint or can bounce off zero
int16_t emotor_command_PWM[NUM_EMOTORS];
int16_t servo_command_PWM[NUM_SERVOS];

uint16_t sbusChannels[16]; // 16 channels on the SBUS

uint32_t blink_counter_microS, blink_delay_microS;
uint32_t checker, current_time_microS;
uint32_t loop_period_microS;
uint32_t previous_time_microS, print_counter_microS;
uint32_t serial_counter_microS;
uint32_t warmup_time_microS;

 #if defined USING_RADIO
   uint32_t channel_failsafe[NUM_CHANNELS], channel_pwm[NUM_CHANNELS], channel_pwm_prev[NUM_CHANNELS];
 # endif

float delta_time;
float faderate;
float low_throttle_cutoff;
float normalised_desired_throttle, normalised_emotor_command[NUM_EMOTORS], normalised_servo_command[NUM_SERVOS], normalised_pot[4];

Euler_angles_degrees desired;
Euler_angles_degrees input, input_rate;
Euler_angles normalised_desired;
Euler_angles passthru;

PID_vals mixed_PID;


/** Externals (yes, I know it's messy but it runs the loop quickly! RMN) **/

extern uint8_t flight_mode;

// IMU:
extern float Accelerometer_gees[3], Accelerometer_gees_prev[3];
extern float Magnetometer[3], Magnetometer_prev[3];
extern float Rate_gyros_degs[3], Rate_gyros_degs_prev[3];

extern Euler_angles_degrees horizontal_IMU, previous_IMU, vertical_IMU;

// BMP280:
extern float Altitude_feet, Altitude_metres;
extern float Pressure_Pa;
extern float Temperature_C;

// Flight models:
extern Euler_angles PID;


//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup()
{
  // Global resets

  armed_to_fly     = false; // Flight status
  fade_up          = false;
  rocket_boost     = false;
  Pid_low_throttle = true;
  warmed_up        = false;

  low_throttle_cutoff = 0.0f; // set later
  

  loop_period_microS = (int) (1.0e6f / loopHz); // (convert to microseconds)

  warmup_time_microS = (int) (1.0f * 1.0e6f); // 1 seconds


  // Setup the aircraft PID gains etc
  Setup_flight_controller();


  //-------------------------------------------------


  #if defined USING_ALTIMETER || defined USE_MPU6050_I2C

    // For the BMPXXX altimeter on I2C bus1, and also the MPU6050 on I2C bus1 if used.
    Wire.begin();

  #endif


  //Serial.begin(500000); // USB serial (rate ignored)
  Serial.begin(9600); // keep the data transfers at low rate or the main loop gets upset

  delay(500);
  

  // Initialise all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify


  #if defined USING_EMOTORS
  
    Serial.println(F("Attaching emotors"));

    #if defined ESC_125_PROTOCOL

      Flight.wentLow = 0;

      // Motor / ESC outputs
      for (ubob = 0; ubob < NUM_EMOTORS; ubob++)
      {
        pinMode(Flight.emotorPin[ubob], OUTPUT);

        Flight.motor_flag[ubob] = 0;
      }

    #elif defined ESC_SERVO_PROTOCOL

      // Pin, min PWM value, max PWM value
      for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
        ESCservo[ubob].attach(emotorPin[ubob], minUs_X450_ESC, maxUs_X450_ESC);

    #endif
  
  #endif
  
  #if defined USING_SERVOS
    Serial.println(F("Attaching servos"));

    setup_servos();
  #endif


  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);


  // Initialise radio communication using SBUS Receiver 
  #if defined USING_RADIO
    Serial.println(F("Initialising radio"));
    sbus.begin();

    // Set radio channels to default (safe) values before entering main loop
    // Radio failsafe values for every channel in the event that bad receiver data is detected.
    // Recommended defaults:
    channel_failsafe[THROTTLE]   = 1000; // about 1000
    channel_failsafe[AILERON]    = 1500; // about 1500
    channel_failsafe[ELEVATOR]   = 1500;
    channel_failsafe[RUDDER]     = 1500;

    channel_failsafe[EMOTOR_CUT] = 2000; // greater than 1500 = throttle cut switch SW6
    channel_failsafe[MODE_MIX]   = 2000; // mode mixer switch                       SW1
    channel_failsafe[IGNITION]   = 1000; // Rocket igniter switch                   SW8
    channel_failsafe[AUX]        = 1000; // Auxiliary switch                        SW2

    channel_failsafe[POT1]       = 1500; // Rotary pot1
    channel_failsafe[POT2]       = 1500; // Rotary pot2
    channel_failsafe[POT3]       = 1500; // Rotary pot3
    channel_failsafe[POT4]       = 1500; // Rotary pot4

    for (uint8_t ubob = 0; ubob < NUM_CHANNELS; ubob++)
      channel_pwm[ubob] = channel_failsafe[ubob];

  #endif


  #if defined USING_ALTIMETER

    // Initialise the BMP280 pressure sensor
    Setup_BMP();

    delay(5);

    // Get initial launchpad/runway altitude
    Read_BMP_values();

  #endif


  // Initialise IMU communication
  Initialise_IMU();

  delay(5);

  // Initialise the remote Artificial horizon and altimeter displays (for debug)
  //Transferacross_setup();

  delay(5);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  //calculate_IMU_error(false); 


  // Arm servo channels
  // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  // Set these to 90 for servos if you do not want them to briefly max out on startup
  #if defined USING_SERVOS
    for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
      servo[ubob].write(90); 
  #endif

  delay(5);


  #if defined USING_EMOTORS

    // Off deadband
    low_throttle_cutoff = 0.04f; // ignore lowest 4% of throttle 

    /* Remove the propellers first!
       Uncomment this function to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle
       to zero after the beeps. Code will not proceed past here if this function is uncommented! */
    // calibrateESCs(); 

    // Arm motors
    for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
      emotor_command_PWM[ubob] = 0; // Command ESC to 0 degrees (limited to 125 later in scalecommands() for 125 protocol)

    ArmEmotors(); // Loop 50 times until Emotor ESCs are happily armed

  #endif


  // Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); // numBlinks, upTime (ms), downTime (ms)

  // If using the MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); // Generates magentometer error and scale factors to be pasted in ProcessImu.cpp
}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop()
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  previous_time_microS = current_time_microS;
  current_time_microS  = micros();

  delta_time = (float) ((current_time_microS - previous_time_microS) * 1.0e-6); // Convert to seconds

  // Warm-up the IMU and give the RC pilot time to sort their life out
  if ((warmed_up == false) && (current_time_microS >  warmup_time_microS))
     warmed_up = true;


  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds


  #if defined PRINT_FOR_DEBUG
    /* Print data at 100 Hz for debugging. Don't uncomment all of these at once! Just one or two,
       otherwise the loop slows down. Note that most of these variables are from the previous loop. */
    if (current_time_microS - print_counter_microS > 10000)
    {
      print_counter_microS = micros();

      // Send IMU data across another I2C interface to separate microcontroller or computer (Teensy has three I2C interfaces)
      //Transferacross_loop();

      //Print_IMU_error(); // Print average of raw data direct from IMU

      // Print filtered data direct from IMU 
      // Accelerometers expected: ~ -2 to 2; x,y 0 when level, z 1 when level Rate gyros expected: ~ -250 to 250, 0 at rest
      //Print_filtered_IMU();   
      //printMagData();      // Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
      //printRollPitchYaw(); // Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
      //Print_BMP_Values();  // Prints values from the BMP280 altimeter code
      //printRadioData();    // Prints radio pwm values (expected: 1000 to 2000)
      //Print_pots();
      //PrintMotorStatus();   // Emotors armed or throttle cut?
      //PrintFlightModes();   // What state is the flight control system in?
      //printDesiredState();  // Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
      //printPIDoutput();     // Prints computed stabilised PID variables from controller and desired setpoint (expected: ~ -1 to 1)
      //printMotorCommands(); // Prints the values being written to the motors (expected: 120 to 250)
      //PrintNormalisedServo();
      //printServoCommands(); // Prints the values being written to the servos (expected: 0 to 180)
      //Print_loop_frequency(); // Prints the actual loop frequency (compare this with loopHz)
      //printLoopRate();
      Serial_plot();
    }
  #endif


  // Get arming status - Check if the throttle cut is off and throttle is low.
  #if defined USING_EMOTORS && defined USING_RADIO
    armedStatus();
  #endif
  

  // Get altimeter data from the BMP280 sensor
  #if defined USING_ALTIMETER
    Read_BMP_values();
  #endif


  // Get vehicle state
  getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise

  // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  // N.B.if no magnetometer data, enter zero for mx, my, mz
  Madgwicks_sensor_fusion( Rate_gyros_degs[STBD_WING_DROPPING], -Rate_gyros_degs[NOSE_DROPPING], -Rate_gyros_degs[HEADING_DECREASING],
                          -Accelerometer_gees[NOSEWARDS], Accelerometer_gees[PORTWARDS], Accelerometer_gees[UPWARDS],
                           Magnetometer[Y], -Magnetometer[X], Magnetometer[Z],
                           delta_time);

  // Decode IMU quaternion output into aircraft Euler angles
  Euler(reallign_zero_pitch);

  // For altitude-dependent code
  Vertical_parameters(delta_time);


  // Normalise the RC pilot's commands (from the previous loop)
  #if defined USING_RADIO
    Get_RCpilot_desired_state();

    // Run the flight controller
    Run_flight_controller(channel_pwm[MODE_MIX]);

  #else
    // Run the flight controller
    Run_flight_controller(1000);
  #endif


  // Scale emotor commands between 125 to 250 range (oneshot125 protocol) and servo PWM commands between 0 to 180 (for servo library)
  scaleCommands();


  // Emotor commands
  #if defined USING_EMOTORS

    // Throttle cut check
    throttleCut(); // Directly sets Emotor commands to low based on state of ch5

    // Give the RC pilot time to sort their life out
    if (warmed_up)
    {
      #if defined ESC_125_PROTOCOL
        CommandEmotors125();           // Sends command pulses to each emotor pin using OneShot125 protocol
      #elif defined ESC_SERVO_PROTOCOL
        for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
          ESCservo[ubob].write(emotor_command_PWM[ubob]); // Writes PWM value as if to servos
      #endif
    }

  #endif


  // Commands to servos - Writes PWM value to servo objects
  #if defined USING_SERVOS

    // Give the RC pilot time to sort their life out
    if (warmed_up)
    {
      for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
        servo[ubob].write(servo_command_PWM[ubob]);
    }

  #endif


  // Get RC pilot's raw commands for the next loop iteration.
  // Prevent failures in event of bad receiver connection - defaults to failsafe values assigned in setup()
  #if defined USING_RADIO
    Get_radio_commands_and_failsafe();        
  #endif


  /* Regulate the loop rate: waits at the end of every loop iteration by sitting in this 'Do' conditional
     until the correct time has passed since the start of the current loop.
     (Interrupt routines running in the background cause the loop rate to fluctuate.) 
  do
  {
  } while (loop_period_microS > (micros() - current_time_microS));*/

  checker = micros();
  
  while (loop_period_microS > (checker - current_time_microS)) 
    checker = micros();
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//


// DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.

#if defined USING_RADIO

void armedStatus()
{
  if ((channel_pwm[EMOTOR_CUT] < 1500) && (channel_pwm[THROTTLE] < 1050))// && (warmed_up))
    armed_to_fly = true;
}


/* DESCRIPTION: Normalises RC pilot-desired control inputs to appropriate values.    
   Updates the desired state variables: normalised_desired_throttle, desired.roll, desired.pitch, and desired.yaw.
   These are computed by using the raw RC commands and normalising them. Also creates passthru.roll, passthru.pitch, and
   passthru.yaw variables, which can be used to control motors/servos with direct unstabilised commands in controlMixer().

   Had to slightly change Nicholas's channel remapping - causing wyrd values in Visual Studio code when using Sbus receiver
   (not as 'arduino environment' as Microsoft claim!)
*/
void Get_RCpilot_desired_state()
{
  float thro_in, roll_in, pitch_in, pot_in[4], yaw_in;

  // Should be between 0 and 1
  thro_in       = channel_pwm[THROTTLE] / 1000.0f - 1.0f; 
  pot_in[ONE]   = channel_pwm[POT1]     / 1000.0f - 1.0f; 
  pot_in[TWO]   = channel_pwm[POT2]     / 1000.0f - 1.0f; 
  pot_in[THREE] = channel_pwm[POT3]     / 1000.0f - 1.0f; 
  pot_in[FOUR]  = channel_pwm[POT4]     / 1000.0f - 1.0f; 

  // Low throttle status flag used in the PID controllers
  Pid_low_throttle = (thro_in < low_throttle_cutoff) ? true : false; // obscure C 'if' statement

  normalised_desired_throttle = LinearInterpolate(thro_in, low_throttle_cutoff, 0.0f, 1.0f, 1.0f); // throttle cut offset

  //Serial.print(F(" normalised_desired_throttle :")); Serial.println(normalised_desired_throttle );

  // Make sure they're limited between 0 and 1
  normalised_pot[ONE]   = constrain(pot_in[ONE],   0.0f, 1.0f);
  normalised_pot[TWO]   = constrain(pot_in[TWO],   0.0f, 1.0f);
  normalised_pot[THREE] = constrain(pot_in[THREE], 0.0f, 1.0f);
  normalised_pot[FOUR]  = constrain(pot_in[FOUR],  0.0f, 1.0f);


  // Should be between -0.5 and 0.5
  roll_in  = channel_pwm[AILERON]  / 1000.0f - 1.5f; 
  pitch_in = channel_pwm[ELEVATOR] / 1000.0f - 1.5f;
  yaw_in   = channel_pwm[RUDDER]   / 1000.0f - 1.5f;

  // Make sure they're limited between -0.5 and 0.5
  passthru.roll  = constrain(roll_in,  -0.5f, 0.5f);
  passthru.pitch = constrain(pitch_in, -0.5f, 0.5f);
  passthru.yaw   = constrain(yaw_in,   -0.5f, 0.5f);

  /*
  Serial.print(F(" pitch_in:"));
  Serial.print(pitch_in);
  Serial.print(F(" passthru.pitch:"));
  Serial.println(passthru.pitch);
  */

  // Doubled, and therefore constrained between -1 and 1
  normalised_desired.roll  = 2.0f * passthru.roll;
  normalised_desired.pitch = 2.0f * passthru.pitch;
  normalised_desired.yaw   = 2.0f * passthru.yaw;
}

#endif


/* DESCRIPTION: Scale normalised actuator commands to values for ESC / Servo protocol.
   normalised_emotor_command[X] variables from the mixer function are scaled to 125-250us for OneShot125 protocol
   or are scaled to 0-180 for the servo protocol

  normalised_servo_command[X] variables from the mixer function are scaled to 0-180 for the servo library using standard PWM.

  emotor_command_PWM[X] are updated here which are used to command the emotors
  servo_command_PWM[X] are updated which are used to command the servos.
  Note that all values below are integers.
  */
void scaleCommands()
{
  // Scale the emotor commands
  for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
  {
    // Scaled to 125us - 250us for oneshot125 protocol
    #if defined ESC_125_PROTOCOL
      emotor_command_PWM[ubob] = (int) (normalised_emotor_command[ubob] * 125.0f + 125.0f);
      emotor_command_PWM[ubob] = constrain(emotor_command_PWM[ubob], 125, 250);

    #elif defined ESC_SERVO_PROTOCOL
      // Scale as if they were servo commands: 0-180
      emotor_command_PWM[ubob] = (int) (normalised_emotor_command[ubob] * 180.0f);
      emotor_command_PWM[ubob] = constrain(emotor_command_PWM[ubob], 0, 180);

    #endif
  }

  // Scale the servo commands: 0-180
  for (uint8_t ubob = 0; ubob < NUM_SERVOS; ubob++)
  {
    servo_command_PWM[ubob] = (int) (normalised_servo_command[ubob] * 180.0f);
    servo_command_PWM[ubob] = constrain(servo_command_PWM[ubob], 0, 180);
  }
}

#if defined USING_RADIO

/* DESCRIPTION: Get PWM values for every channel from an SBUS radio receiver.
   The values are pulled from the SBUS library directly.
   The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
 */
void Get_radio_commands_and_failsafe()
{
  uint32_t minVal =  800;
  uint32_t maxVal = 2200;

  uint8_t check[7];

  // Reset
  check[THROTTLE] = check[AILERON] = check[ELEVATOR] = check[RUDDER] = check[EMOTOR_CUT] = check[MODE_MIX] = check[IGNITION] = 0;


  // Reading SBUS receiver (if connection is okay)
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame) )
  {

  #if defined TARANIS_PLUS_RX || defined X4R_SB_RX // For Taranis-Plus and X4R-SB

    // Linear remapping, equation of a line: Y=mx+c
    float scale =  0.615f; // gradient m
    float bias = 895.0f;   // constant (offset) c

    channel_pwm[THROTTLE] = (uint32_t) (sbusChannels[ONE]   * scale + bias); // Throttle 1000 min, 2000 max  
    channel_pwm[AILERON]  = (uint32_t) (sbusChannels[TWO]   * scale + bias); // Roll 1000 full left, 2000 full right, 1500 center  
    channel_pwm[ELEVATOR] = (uint32_t) (sbusChannels[THREE] * scale + bias); // Pitch 1000 pitch up, 2000 pitch down, 1500 center
    channel_pwm[RUDDER]   = (uint32_t) (sbusChannels[FOUR]  * scale + bias); // Yaw 2000 full left, 1000 full right, 1500 center 

  #elif defined GRAUPNER_HOTT_GR12L_RX // For Graupner Hott GR12L SBUS receiver. Rick Newlands (thanks Dad!)

    // Linear remapping, equation of a line: Y=mx+c
    float scale =  0.784f; // gradient m
    float bias = 697.492f; // constant (offset) c

    // Graupner Hott GR12L SBUS receiver - note that channels map differently:
    channel_pwm[THROTTLE] = (uint32_t) (sbusChannels[THREE] * scale + bias); // Throttle 1000 min, 2000 max  
    channel_pwm[AILERON]  = (uint32_t) (sbusChannels[ONE]   * scale + bias); // Roll 1000 full left, 2000 full right, 1500 center  
    channel_pwm[ELEVATOR] = (uint32_t) (sbusChannels[TWO]   * scale + bias); // Pitch 1000 pitch up, 2000 pitch down, 1500 center 
    channel_pwm[RUDDER]   = (uint32_t) (sbusChannels[FOUR]  * scale + bias); // Yaw 2000 full left, 1000 full right, 1500 center 

  #endif

    // Auxiliary channels:
    channel_pwm[EMOTOR_CUT] = (uint32_t) (sbusChannels[FIVE]  * scale + bias); // Throttle cut when greater than 1500
    channel_pwm[MODE_MIX]   = (uint32_t) (sbusChannels[SIX]   * scale + bias); // Used for flight mode mixing
    channel_pwm[IGNITION]   = (uint32_t) (sbusChannels[SEVEN] * scale + bias); // Rocketry ignition switch
    channel_pwm[AUX]        = (uint32_t) (sbusChannels[EIGHT] * scale + bias); // Auxiliary switch

    channel_pwm[POT1]       = (uint32_t) (sbusChannels[NINE]   * scale + bias); // Rotary potentiometer 1
    channel_pwm[POT2]       = (uint32_t) (sbusChannels[TEN]    * scale + bias); // Rotary potentiometer 2
    channel_pwm[POT3]       = (uint32_t) (sbusChannels[ELEVEN] * scale + bias); // Rotary potentiometer 3
    channel_pwm[POT4]       = (uint32_t) (sbusChannels[TWELVE] * scale + bias); // Rotary potentiometer 4
  }


  /* Failsafe: If radio gives garbage values, set all commands to default values
  Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values.
  If any of the commands are lower than 800 or higher than 2200, then we can be certain that there is an
  issue with the radio connection (most likely hardware related).
  If any of the channels show this failure, then all of the radio commands channel_x_pwm are set to
  default failsafe values specified in the setup. 
  Comment out this function when troubleshooting your radio connection in case any extreme values are
  triggering this function to overwrite the printed variables.
   */

  // Triggers for failure criteria
  if (channel_pwm[THROTTLE]   > maxVal || channel_pwm[THROTTLE]   < minVal) check[THROTTLE]   = 1;
  if (channel_pwm[AILERON]    > maxVal || channel_pwm[AILERON]    < minVal) check[AILERON]    = 1;
  if (channel_pwm[ELEVATOR]   > maxVal || channel_pwm[ELEVATOR]   < minVal) check[ELEVATOR]   = 1;
  if (channel_pwm[RUDDER]     > maxVal || channel_pwm[RUDDER]     < minVal) check[RUDDER]     = 1;

  if (channel_pwm[EMOTOR_CUT] > maxVal || channel_pwm[EMOTOR_CUT] < minVal) check[EMOTOR_CUT] = 1;
  if (channel_pwm[MODE_MIX]   > maxVal || channel_pwm[MODE_MIX]   < minVal) check[MODE_MIX]   = 1;
  if (channel_pwm[IGNITION]   > maxVal || channel_pwm[IGNITION]   < minVal) check[IGNITION]   = 1;


  // If any failures, set to default failsafe values
  if ((check[THROTTLE] + check[AILERON] + check[ELEVATOR] + check[RUDDER] + check[EMOTOR_CUT] + check[MODE_MIX] + check[IGNITION]) > 0)
  {
    channel_pwm[THROTTLE]   = channel_failsafe[THROTTLE];
    channel_pwm[AILERON]    = channel_failsafe[AILERON];
    channel_pwm[ELEVATOR]   = channel_failsafe[ELEVATOR];
    channel_pwm[RUDDER]     = channel_failsafe[RUDDER];
    channel_pwm[EMOTOR_CUT] = channel_failsafe[EMOTOR_CUT];
    channel_pwm[MODE_MIX]   = channel_failsafe[MODE_MIX];
    channel_pwm[IGNITION]   = channel_failsafe[IGNITION];
  }
  else /*** normal operation ***/
  {
    // Low-pass the critical commands and update previous values.
    low_pass_filter_UL(&channel_pwm[THROTTLE], &channel_pwm_prev[THROTTLE], 0.7f);
    low_pass_filter_UL(&channel_pwm[AILERON],  &channel_pwm_prev[AILERON],  0.7f);
    low_pass_filter_UL(&channel_pwm[ELEVATOR], &channel_pwm_prev[ELEVATOR], 0.7f);
    low_pass_filter_UL(&channel_pwm[RUDDER],   &channel_pwm_prev[RUDDER],   0.7f);

    // Channels 5 to 8 aren't filtered, as they're either 'on' or 'off'.

    // Channels 9 to 12 are rotary potentiometers, which can glitch
    low_pass_filter_UL(&channel_pwm[POT1], &channel_pwm_prev[POT1], 0.7f);
    low_pass_filter_UL(&channel_pwm[POT2], &channel_pwm_prev[POT2], 0.7f);
    low_pass_filter_UL(&channel_pwm[POT3], &channel_pwm_prev[POT3], 0.7f);
    low_pass_filter_UL(&channel_pwm[POT4], &channel_pwm_prev[POT4], 0.7f);
  }
}

#endif


/* DESCRIPTION: Send pulses to motor pins, Oneshot125 protocol
   Nicholas Rehm's implementation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin).
   The pulse lengths being sent are mX_command_PWM, computed in scaleCommands().  */
void CommandEmotors125()
{
#if defined ESC_125_PROTOCOL

  uint32_t pulseStart, timer;
  
  uint8_t wentLow;

  bool motor_flag[NUM_EMOTORS];

  wentLow = 0; // Reset
  
  // Write all motor pins high
  for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
    digitalWrite(emotorPin[ubob], HIGH);

  pulseStart = micros();

  // Write each motor pin low as correct pulse length is reached.
  // Keep going until final pulse is finished, then done.
  while (wentLow < NUM_EMOTORS) 
  {
    timer = micros() - pulseStart;

    if ((emotor_command_PWM[ONE] <= timer) && (motor_flag[ONE] == false)) 
    {
      digitalWrite(emotorPin[ONE], LOW);

      wentLow += 1;

      motor_flag[ONE] = true;
    }

    if ((emotor_command_PWM[TWO] <= timer) && (motor_flag[TWO] == false))
    {
      digitalWrite(emotorPin[TWO], LOW);

      wentLow += 1;

      motor_flag[TWO] = true;
    }

    if ((emotor_command_PWM[THREE] <= timer) && (motor_flag[THREE] == false))
    {
      digitalWrite(emotorPin[THREE], LOW);

      wentLow += 1;

      motor_flag[THREE] = true;
    }

    /*
    if ((emotor_command_PWM[FOUR] <= timer) && (motor_flag[FOUR] == false))
    {
      digitalWrite(emotorPin[FOUR], LOW);

      wentLow += 1;

      motor_flag[FOUR] = true;
    }

    if ((emotor_command_PWM[FIVE] <= timer) && (motor_flag[FIVE] == false)) 
    {
      digitalWrite(emotorPin[FIVE], LOW);

      wentLow += 1;

      motor_flag[FIVE] = true;
    } 

    if ((emotor_command_PWM[SIX] <= timer) && (motor_flag[SIX] == false)) 
    {
      digitalWrite(emotorPin[SIX], LOW);

      wentLow += 1;

      motor_flag[SIX] = true;
    } 
    */
  }
  #endif
}


#if defined USING_RADIO

// DESCRIPTION: Sends 50 command pulses to the Emotors, to be used to arm Emotors in the void setup()
void ArmEmotors() 
{  
  /* Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
     function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
     for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++)
  {
    #if defined ESC_125_PROTOCOL
      commandMotors125(); // Sends command pulses to each emotor pin using OneShot125 protocol

    #elif defined ESC_SERVO_PROTOCOL
      for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
        ESCservo[ubob].write(emotor_command_PWM[ubob]); // Writes PWM value as if to servos

    #endif

    delay(2);
  }
}

/* Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
   Simulates the void loop(), but only for the purpose of providing throttle pass-through to the motors,
   so that you can power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero.
   This function should only be uncommented when performing an ESC calibration. */
void calibrateESCs()
{
  while (true)
  {
    previous_time_microS = current_time_microS;
    current_time_microS  = micros();

    delta_time = (float) ((current_time_microS - previous_time_microS) * 1.0e-6); // Convert to seconds

    // LED on to indicate we are NOT in the main loop
    digitalWrite(13, HIGH); 


    Get_radio_commands_and_failsafe();

    Get_RCpilot_desired_state();

    getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise

    // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
    // N.B.if no magnetometer data, enter zero for mx, my, mz
    Madgwicks_sensor_fusion( Rate_gyros_degs[STBD_WING_DROPPING], -Rate_gyros_degs[NOSE_DROPPING], -Rate_gyros_degs[HEADING_DECREASING],
                            -Accelerometer_gees[NOSEWARDS], Accelerometer_gees[PORTWARDS], Accelerometer_gees[UPWARDS],
                             Magnetometer[Y], -Magnetometer[X], Magnetometer[Z],
                             delta_time);

    Euler(reallign_zero_pitch);

    Get_RCpilot_desired_state();
      
    for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
      normalised_emotor_command[ubob] = normalised_desired_throttle;

    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
    //throttleCut(); // Directly sets motor commands to low based on state of ch5
      
    #if defined ESC_125_PROTOCOL
      CommandEmotors125(); // Sends command pulses to each emotor pin using OneShot125 protocol
    #elif defined ESC_SERVO_PROTOCOL
      for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
        ESCservo[ubob].write(emotor_command_PWM[ubob]); // Writes PWM value as if to servos
    #endif
      
    printRadioData(); // Radio pwm values (expected: 1000 to 2000)
      
      /* Regulate the loop rate: waits at the end of every loop iteration by sitting in this 'Do' conditional
     until the correct time has passed since the start of the current loop.
     (Interrupt routines running in the background cause the loop rate to fluctuate.) 
    do
    {
    } while (loop_period_microS > (micros() - current_time_microS));*/

    uint32_t checker = micros();
  
    while (loop_period_microS > (checker - current_time_microS)) 
      checker = micros();
  }

}

// DESCRIPTION: Directly set actuator outputs to minimum value if triggered
void throttleCut()
{

  /* Monitors the state of radio command channel_pwm[EMOTOR_CUT] and directly sets the mx_command_PWM values to minimum (120 is
     minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel_pwm[EMOTOR_CUT] is high.
     This is the last function called before commandMotors() is called so that the last thing checked is if the user
     is giving permission to command the motors to anything other than minimum value. Safety first!

     channel_pwm[EMOTOR_CUT] is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
     channel_pwm[EMOTOR_CUT] is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */

  // Latch armed_to_fly off
  if ((channel_pwm[EMOTOR_CUT] > 1500) || (armed_to_fly == false))
  {
    armed_to_fly = false;

    for (uint8_t ubob = 0; ubob < NUM_EMOTORS; ubob++)
    {
      #if defined ESC_125_PROTOCOL
        emotor_command_PWM[ubob] = 120;
      #elif defined ESC_SERVO_PROTOCOL
        emotor_command_PWM[ubob] = 0; // using servo PWM variables to control motor ESCs
      #endif
    }
  }
}

#endif


// DESCRIPTION: Blink LED on board to indicate main loop is running
void loopBlink()
{
  /* It looks cool. */
  if (current_time_microS - blink_counter_microS > blink_delay_microS)
  {
    blink_counter_microS = micros();

    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED
    
    if (blinkAlternate == 1)
    {
      blinkAlternate = 0;
      blink_delay_microS = 100000;
    }
    else if (blinkAlternate == 0)
    {
      blinkAlternate = 1;
      blink_delay_microS = 2000000;
    }
  }
}

// DESCRIPTION: Simple function to make LED on board blink as desired
void setupBlink(int numBlinks,int upTime, int downTime)
{
  
  for (int j = 1; j<= numBlinks; j++)
  {
    digitalWrite(13, LOW);
    delay(downTime);

    digitalWrite(13, HIGH);
    delay(upTime);
  }
}




//========================================================================================================================//

#if defined USING_RADIO

void printRadioData()
{
  Serial.print(F(" Throttle:"));
  Serial.print(channel_pwm[THROTTLE]);

  Serial.print(F(" Aileron:"));
  Serial.print(channel_pwm[AILERON]);

  Serial.print(F(" Elevator:"));
  Serial.print(channel_pwm[ELEVATOR]);

  Serial.print(F(" Rudder:"));
  Serial.print(channel_pwm[RUDDER]);

  Serial.print(F(" MOTOR_CUT:"));
  Serial.print(channel_pwm[EMOTOR_CUT]);

  Serial.print(F(" MODE_MIX:"));
  Serial.println(channel_pwm[MODE_MIX]);

  Serial.print(F(" IGNITION:"));
  Serial.println(channel_pwm[IGNITION]);

  Serial.print(F(" AUX:"));
  Serial.println(channel_pwm[AUX]);

  Serial.print(F(" POT1:"));
  Serial.print(channel_pwm[POT1]);

  Serial.print(F(" POT2:"));
  Serial.print(channel_pwm[POT2]);

  Serial.print(F(" POT3:"));
  Serial.print(channel_pwm[POT3]);

  Serial.print(F(" POT4:"));
  Serial.print(channel_pwm[POT4]);

  Serial.println();
}

void Print_pots()
{
  Serial.print(F(" POT1:"));
  Serial.print(normalised_pot[ONE]);

  Serial.print(F(" POT2:"));
  Serial.print(normalised_pot[TWO]);

  Serial.print(F(" POT3:"));
  Serial.print(normalised_pot[THREE]);

  Serial.print(F(" POT4:"));
  Serial.print(normalised_pot[FOUR]);

  Serial.println();
}

void PrintMotorStatus()
{
  if (armed_to_fly)
    Serial.print(F(" Motors armed"));

  if (channel_pwm[EMOTOR_CUT] > 1500)
    Serial.print(F(" Throttle cut"));

  Serial.println();
}

#endif

void printDesiredState()
{
  Serial.print(F("thro_desired:"));
  Serial.print(normalised_desired_throttle);

  Serial.print(F(" roll_desired:"));
  Serial.print(desired.roll_degs);
  
  Serial.print(F(" pitch_desired:"));
  Serial.print(desired.pitch_degs);
  
  Serial.print(F(" yaw_desired:"));
  Serial.println(desired.yaw_degs);
}

void printRollPitchYaw()
{
  Serial.print(F("h_roll:"));
  Serial.print(horizontal_IMU.roll_degs);

  Serial.print(F("v_roll:"));
  Serial.print(vertical_IMU.roll_degs);

  Serial.print(F(" h_pitch:"));
  Serial.print(horizontal_IMU.pitch_degs);

  Serial.print(F(" v_pitch:"));
  Serial.print(vertical_IMU.pitch_degs);

  Serial.print(F(" h_yaw:"));
  Serial.print(horizontal_IMU.yaw_degs);

  Serial.print(F(" v_yaw:"));
  Serial.println(vertical_IMU.yaw_degs);
}

void Print_BMP_Values()
{
  Serial.print(F("Temperature: "));
  Serial.print(Temperature_C);
  Serial.println(F(" °C"));

  Serial.print(F("Pressure:    "));
  Serial.print(Pressure_Pa);
  Serial.println(F(" Pa"));

  Serial.print(F("Altitude:    "));
  Serial.print(Altitude_metres);
  Serial.println(F(" m"));

  Serial.println();
}

void printMotorCommands()
{
  Serial.print(F("emotor1:"));
  Serial.print(emotor_command_PWM[ONE]);

  Serial.print(F(" emotor2:"));
  Serial.print(emotor_command_PWM[TWO]);

  Serial.print(F(" emotor3:"));
  Serial.print(emotor_command_PWM[THREE]);

  /*
  Serial.print(F(" emotor4:"));
  Serial.print(emotor_command_PWM[FOUR]);

  Serial.print(F(" emotor5:"));
  Serial.print(emotor_command_PWM[FIVE]));

  Serial.print(F(" emotor6:"));
  Serial.print(emotor_command_PWM[SIX]);
  */

  Serial.println();
}

void printServoCommands()
{
  Serial.print(F("servo1:"));
  Serial.print(servo_command_PWM[ONE]);

  Serial.print(F(" servo2:"));
  Serial.print(servo_command_PWM[TWO]);

  Serial.print(F(" servo3:"));
  Serial.print(servo_command_PWM[THREE]);

  Serial.print(F(" servo4:"));
  Serial.print(servo_command_PWM[FOUR]);

  /*
  Serial.print(F(" servo5:"));
  Serial.print(servo_command_PWM[FIVE]));

  Serial.print(F(" servo6:"));
  Serial.print(servo_command_PWM[SIX]);
  */

  Serial.println();
}

// Error averaged over 12000 cycles
void Print_IMU_error()
{
  Serial.print(F("AccelX:"));
  Serial.print(Accelerometer_gees[NOSEWARDS]);

  Serial.print(F(" AccelY:"));
  Serial.print(Accelerometer_gees[PORTWARDS]);

  Serial.print(F(" AccelZ:"));
  Serial.print(Accelerometer_gees[UPWARDS]);

  Serial.print(F(" GyroX:"));
  Serial.print(Rate_gyros_degs[STBD_WING_DROPPING]);

  Serial.print(F(" GyroY:"));
  Serial.print(Rate_gyros_degs[NOSE_DROPPING]);

  Serial.print(F(" GyroZ:"));
  Serial.println(Rate_gyros_degs[HEADING_DECREASING]);
}

void Print_filtered_IMU()
{
  Serial.print(F("AccelX:"));
  Serial.print(Accelerometer_gees[NOSEWARDS]);

  Serial.print(F(" AccelY:"));
  Serial.print(Accelerometer_gees[PORTWARDS]);

  Serial.print(F(" AccelZ:"));
  Serial.print(Accelerometer_gees[UPWARDS]);

  Serial.print(F(" GyroX:"));
  Serial.print(Rate_gyros_degs[STBD_WING_DROPPING]);

  Serial.print(F(" GyroY:"));
  Serial.print(Rate_gyros_degs[NOSE_DROPPING]);

  Serial.print(F(" GyroZ:"));
  Serial.println(Rate_gyros_degs[HEADING_DECREASING]);
}

// Print the actually achieved loop frequency versus the desired one
void Print_loop_frequency()
{
  Serial.print(F("Loop frequency:"));
  Serial.println(1.0f / delta_time);
}

void printLoopRate()
{
  Serial.print(F("delta_time:"));
  Serial.println(delta_time, 6);
}

