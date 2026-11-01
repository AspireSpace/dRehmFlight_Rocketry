// dRehmFlight_Rocketry project header file for Visual Studio Code
// Rick Newlands March 2025
 
// N.B. In the Arduino environment, any variable declared outside of a function (e.g. setup(), loop(), etc. ),
// is a global variable. Global variables are static.


#ifndef _DREHMFLIGHT_ROCKETRY_H_
#define _REHMFLIGHT_ROCKETRY_H_

#include <Arduino.h>

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//



// Uncomment only one IMU
#define USE_MPU6050_I2C // Default
//#define USE_MPU9250_SPI


// For transition fading
#define FADE_UP    1.0f
#define FADE_DOWN -1.0f


/* float-precision constants and conversion factors.
   #define saves internal memory on the microcontroller, whereas 'const' doesn't, but it can cause problems. */
#define G_METRICf    9.80665f      // 1 gravity
#define DEG_TO_RADf  0.017453292f
#define RAD_TO_DEGf 57.29577951f
#define HALF_PIf     1.57079633f

#define FEET_TO_METRESf	0.3048f // exactly
#define METRES_TO_FEETf	(1.0f / FEET_TO_METRESf)

#define KELVIN_TO_CENTIGRADEf -273.15f // to Centigrade
#define CENTIGRADE_TO_KELVINf  273.15f // to Kelvin


/* Enumerated types for the benefit of us humans - the compiler just codes them as a
   positive integer from zero to whatever in ascending numerical order .
   (Note that the last entry never has a comma after it.) */

enum radio_channel_type // (alter to suit your own radio)
{
  THROTTLE,
  AILERON,
  ELEVATOR,
  RUDDER,
  EMOTOR_CUT,
  MODE_MIX,
  IGNITION,
  AUX,
  POT1,
  POT2,
  POT3,
  POT4
};

enum PID_type
{
  CONTROL_ANGLE,
  CONTROL_RATE
};

enum accelerometer_range_type // full scale accelerometer range (g's)
{
  ACCEL_2G, // Default
  ACCEL_4G,
  ACCEL_8G,
  ACCEL_16G // Rocketry
};

enum gyro_range_type // full scale rate gyro range (deg/sec)
{
  GYRO_250DPS, // Default
  GYRO_500DPS, // For dRehmFlight's F35 code
  GYRO_1000DPS,
  GYRO_2000DPS
};

enum re_index_type // because C++ arrays count up from 0 rather than 1 - annoying!
{
  ONE,
  TWO,
  THREE,
  FOUR,
  FIVE,
  SIX,
  SEVEN,
  EIGHT,
  NINE,
  TEN,
  ELEVEN,
  TWELVE
};

enum accels_type
{
  NOSEWARDS,
  PORTWARDS, // to port side
  UPWARDS
};

enum gyros_type
{
  STBD_WING_DROPPING, // starboard wing dropping
  NOSE_DROPPING,
  HEADING_DECREASING // compass heading decreasing
};

enum mags_type // TBD, don't know orientations
{
  X,
  Y,
  Z
};


/* Structures allow you to group variables and types together */
struct Euler_angles
{
  float roll;
  float pitch;
  float yaw;
};

struct Euler_angles_degrees
{
  float roll_degs;
  float pitch_degs;
  float yaw_degs;
};

struct PID_vals
{
  Euler_angles Kd;
  Euler_angles Ki;
  Euler_angles Kp;

  Euler_angles max;
};


class Fader // A 'class' is an enhanced 'struct' with its own functions and subroutines
{
  public:
    bool transitioning;
    
    //uint32_t start_microS;

    float delta_param;
    float fade_time;
    float mixer;

    void initialise(float fade_direction);

    void trigger(float fade_direction);

    void ramp(void);

    float Mix(float param1, float param2);
};




//========================================================================================================================//
//                                                   Function defines RMN                                                 //                           
//========================================================================================================================//

void armedStatus(void);

void Initialise_IMU(void);

void getIMUdata(void);

void calculate_IMU_error(bool vertical);

void Madgwicks_sensor_fusion(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float d_time);

void Euler(bool add_ninety);

void Vertical_parameters(float d_time);

void Estimate_seaLevel_conditions(float site_pressure_Pa, float site_temperature_C, float site_elevation_m);

float Calculate_altitude(float pressure_at_altitude_Pa);

void Setup_BMP(void);

void Read_BMP_values(void);

void Transferacross_setup(void);

void Transferacross_loop(void);

void setup_servos(void);

void Setup_flight_controller(void);

void Initialise_Rocketry(void);
 
void Get_RCpilot_desired_state(void);

void Run_flight_controller(uint32_t mode_mixer);

void Rocketry_Computer(float d_time);

float Hover_PID(float d_time);

void scaleCommands(void);

void Get_radio_commands_and_failsafe(void);

void CommandEmotors125(void);

void ArmEmotors(void);

void calibrateESCs(void);

void throttleCut(void);

void IMU_orientation(Fader reorientate, bool drone_mode);

void AttitudeController(uint8_t stabilisation_mode, bool safety_reset_PID_I, bool tune_the_PIDS);

void low_pass_filter_UL(unsigned long *current, unsigned long *previous, float weighting);

float LinearInterpolate(float x, float x1, float y1, float x2, float y2);

void Reverse_servo(float *normalised_servo_value);

void Serial_plot(void);

void loopBlink(void);

void setupBlink(int numBlinks,int upTime, int downTime);

void printRadioData(void);

void Print_pots(void);

void PrintMotorStatus(void);

void PrintFlightModes(void);

void printDesiredState(void);

void Print_loop_frequency(void);

void Print_IMU_error(void);
  
void Print_filtered_IMU(void); 

void printRollPitchYaw(void);

void Print_BMP_Values(void);

void printPIDoutput(void);

void printMotorCommands(void);

void PrintNormalisedServo(void);

void printServoCommands(void);

void printLoopRate(void);


#endif /* _DREHMFLIGHT_ROCKETRY_H_ */