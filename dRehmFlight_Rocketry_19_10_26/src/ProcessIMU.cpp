/* ProcessIMU.cpp 

   Rick Newlands April 26

   Thanks to Nicholas Rehm's dRehmFlight, and Dronebot Workshop

   Avoid cheap 6050 chips - they're fake, have mismatched internal registers, and the gyros drift like buggery!

   N.B! Old versions of i2cdevlib don't work on modern ESP32's: get Jeff Rowberg's latest version (2022 or later) e.g.
   https://github.com/jrowberg/i2cdevlib/blob/98a3b4ec838223fd5316de54203a6dc88718cd00/Arduino/I2Cdev/I2Cdev.h


   MPU9250 implementation based on MPU9250 library by:
   brian.taylor@bolderflight.com
   http://www.bolderflight.com  

   Version 2 - magnetometer code reformulated and checked 6/6/26 RMN
*/

#include <Arduino.h>           // for standard typedefs etc
#include <Wire.h>               // I2C communication
#include <SPI.h>                 // SPI communication      
#include "dRehmFlight_Rocketry.h" // project header file Rick Newlands



#if defined USE_MPU6050_I2C
  #include "MPU6050.h" // which itself includes Jeff Rowberg's I2C interface

  // Declare an instance of the MPU6050 IMU chip on the primary I2C bus
  MPU6050 mpu6050;

#elif defined USE_MPU9250_SPI // includes a magnetometer

  #include "MPU9250.h"

  // Declare an instance of the MPU9250 IMU chip on the SPI2 bus
  MPU9250 mpu9250(SPI2,36);

#endif


/** Global variables (these are static in the Arduino environment) **/

/* IMU calibration parameters - calibrate your own IMU chip using subroutine calculate_IMU_error() below.
   Then comment-out calculate_IMU_error() again */
float Accelerometer_gees_Error[3] = {0.0f, 0.0f, 0.0f};
float Rate_gyros_degs_Error[3]    = {0.0f, 0.0f, 0.0f};

/* Magnetometer calibration parameters - calibrate your own IMU chip using subroutine calibrateMagnetometer() below.
   Then comment-out calibrateMagnetometer() again */
float MagError[3] = {0.0f, 0.0f, 0.0f};
float MagScale[3] = {1.0f, 1.0f, 1.0f};


// Filter parameters: Nicholas Rehm's default values for a loop frequency of 2kHz
float B_madgwick = 0.04f; // Madgwick filter parameter
float B_accel    = 0.14f; // Accelerometer LP filter paramter, (MPU6050 default: 0.14f, MPU9250 default: 0.20f)
float B_gyro     = 0.1f;  // Gyro LP filter paramter,          (MPU6050 default: 0.10f, MPU9250 default: 0.17f)
//float B_mag      = 1.0f;  // Magnetometer LP filter parameter

// The square of max acceptable gees (4 gees) that won't washout the gravity data
float max_gee_squared;

// Initialisation parameters
uint8_t ACCEL_SCALE;
uint8_t GYRO_SCALE;
uint8_t IMU_accelerometer_range, IMU_rate_gyro_range;

float ACCEL_SCALE_FACTOR;
float GYRO_SCALE_FACTOR;

// Quaternian array (four parameters)
float quat[4];

float Accelerometer_gees[3], Accelerometer_gees_prev[3], altitude_AMSL;
float height_metres;
float launch_altitude_metres;
float old_upward_accel_metres_per_sec2, old_upward_velocity_metres_per_sec;
float Magnetometer[3], Magnetometer_prev[3];
float Rate_gyros_degs[3], Rate_gyros_degs_prev[3];
float upward_accel_metres_per_sec2, upward_velocity_metres_per_sec;

Euler_angles_degrees horizontal_IMU, previous_IMU, vertical_IMU;


/** Local functions **/
void low_pass_filter(float *current, float *previous, float weighting);

float Safe_arcsinf(float value);
float Integrate(float older, float newer, float interval);


/** Externals (yes, I know it's messy but it runs the loop quickly! RMN) **/
extern bool tailsitting;


// Initialise the IMU's interface chip
void Initialise_IMU()
{
  // Zero various parameters
  for (uint8_t ii = 0; ii < 3; ii++)
  {
    Accelerometer_gees[ii]      = Rate_gyros_degs[ii]      = Magnetometer[ii]      = 0.0f;
    Accelerometer_gees_prev[ii] = Rate_gyros_degs_prev[ii] = Magnetometer_prev[ii] = 0.0f;
  }

  old_upward_accel_metres_per_sec2 = upward_accel_metres_per_sec2 = old_upward_velocity_metres_per_sec = upward_velocity_metres_per_sec = 0.0f;
  height_metres = launch_altitude_metres = 0.0f;


  // Setup accelerometer full-scale value selection, and range-scaling factor
  switch(IMU_accelerometer_range)
  {
    default:
    case ACCEL_2G:

    #if defined USE_MPU6050_I2C
      ACCEL_SCALE = MPU6050_ACCEL_FS_2;
    #elif defined USE_MPU9250_SPI
      ACCEL_SCALE = mpu9250.IMU_accelerometer_range_2G
    #endif

      ACCEL_SCALE_FACTOR = 16384.0f;
    break;


    case ACCEL_4G:

    #if defined USE_MPU6050_I2C
      ACCEL_SCALE = MPU6050_ACCEL_FS_4;
    #elif defined USE_MPU9250_SPI
      ACCEL_SCALE = mpu9250.IMU_accelerometer_range_4G
    #endif

      ACCEL_SCALE_FACTOR = 8192.0f;
    break;


    case ACCEL_8G:

    #if defined USE_MPU6050_I2C
      ACCEL_SCALE = MPU6050_ACCEL_FS_8;
    #elif defined USE_MPU9250_SPI
      ACCEL_SCALE = mpu9250.IMU_accelerometer_range_8G
    #endif

      ACCEL_SCALE_FACTOR = 4096.0f;
    break;


    case ACCEL_16G:

    #if defined USE_MPU6050_I2C
      ACCEL_SCALE = MPU6050_ACCEL_FS_16;
    #elif defined USE_MPU9250_SPI
      ACCEL_SCALE = mpu9250.IMU_accelerometer_range_16G
    #endif

      ACCEL_SCALE_FACTOR = 2048.0f;
    break;
  }


  // Setup rate-gyro full-scale value selection, and range-scaling factor
  switch (IMU_rate_gyro_range)
  {
    default:
    case GYRO_250DPS:

    #if defined USE_MPU6050_I2C
      GYRO_SCALE = MPU6050_GYRO_FS_250;
    #elif defined USE_MPU9250_SPI
      GYRO_SCALE = mpu9250.GYRO_RANGE_250DPS
    #endif

      GYRO_SCALE_FACTOR = 131.0f;
    break;


    case GYRO_500DPS:

    #if defined USE_MPU6050_I2C
      GYRO_SCALE = MPU6050_GYRO_FS_500;
    #elif defined USE_MPU9250_SPI
      GYRO_SCALE = mpu9250.GYRO_RANGE_500DPS
    #endif

      GYRO_SCALE_FACTOR = 65.5f;
    break;


    case GYRO_1000DPS:

    #if defined USE_MPU6050_I2C
      GYRO_SCALE = MPU6050_GYRO_FS_1000;
    #elif defined USE_MPU9250_SPI
      GYRO_SCALE = mpu9250.GYRO_RANGE_1000DPS
    #endif

      GYRO_SCALE_FACTOR = 32.8f;
    break;


    case GYRO_2000DPS:

    #if defined USE_MPU6050_I2C
      GYRO_SCALE = MPU6050_GYRO_FS_2000;
    #elif defined USE_MPU9250_SPI
      GYRO_SCALE = mpu9250.GYRO_RANGE_2000DPS
    #endif

      GYRO_SCALE_FACTOR = 16.4f;
    break;
  }


  /* Initialise the Quaternion (see below) to values that assume the aircraft body axes
     are aligned with Earth axes. */
  if (tailsitting) // Aircraft (and its IMU chip) initially on a vertical launchpad (tail-sitting)?
  {
    quat[0] = quat[2] = sqrtf(0.5f);
  }
  else // Aircraft initially on a flat horizontal runway, with the IMU chip parallel to the ground
  {
    quat[0] = 1.0f;
    quat[2] = 0.0f;
  }

  quat[1] = quat[3] = 0.0f;


  // Gees above which the gravity signal could be washed-out.
  max_gee_squared = sq(4.0f);


  /* Using the MPU 6050 IMU chip */
  #if defined USE_MPU6050_I2C

    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

  // Call to initialise the MPU6050 IMU:
  // Wake it up by writing zero to its Power management register (0x6B). It loves to stay asleep (who doesn't!)
  // This call may also reset all sensors to zero.
  // Also appears to do something magic to its internal noise filters!
  // Pre-2022 i2C library will fail here if using an ESP32
  mpu6050.initialize();
  
  // Check that it's happy:
  // When the WHO_AM_I register (0x75) is read, it should return 0x34 hex unless it's a fake chip.
  // (It's a 6-bit register instead of 8 bits; this generally catches the counterfeiters out!)
  if (mpu6050.testConnection() == false)
  {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {} // stay stuck in loop here
  }


  // Setup rate-gyros full-scale value selection and scale factor
  // Set bits 4 and 5 of the GYRO_CONFIG_REGISTER (0x1B) register thusly:
  //                          bits: (876)54(321)
  // 0 = +/- 250 degrees/sec  = 00 = 000 00 000 = 0x00 hex (= default)
  // 1 = +/- 500 degrees/sec  = 01 = 000 01 000 = 0x08 hex
  // 2 = +/- 1000 degrees/sec = 10 = 000 10 000 = 0x10 hex
  // 3 = +/- 2000 degrees/sec = 11 = 000 11 000 = 0x18 hex
  //                                     --
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // this call just sets bits 4 and 5 alone


  // Setup accelerometer full-scale value selection, and scale factor
  // This sets bits 4 and 5 of the ACCEL_CONFIG_REGISTER (0x1C) thusly:
  //             bits: (876)54(321)
  // 0 = +/- 2g  = 00 = 000 00 000 bin = 0x00 hex (= default)
  // 1 = +/- 4g  = 01 = 000 01 000 bin = 0x08 hex
  // 2 = +/- 8g  = 10 = 000 10 000 bin = 0x10 hex
  // 3 = +/- 16g = 11 = 000 11 000 bin = 0x18 hex
  //                        --
  mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // this call just sets bits 4 and 5 alone


  /* Using the MPU 9250 IMU chip */
  #elif defined USE_MPU9250_SPI

    // Call to initialise the MPU9250 IMU: 
    int status = mpu9250.begin();    

    if (status < 0)
    {
      Serial.println("MPU9250 initialisation unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: "); Serial.println(status);
      while(1) {} // stay stuck in loop here
    }

    /* From the reset state all registers should be 0x00, so we should be at max sample rate with 
       digital low pass filter(s) off. All we need to do is set the desired fullscale ranges. */
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);

    // Input the calibration factors (see calibrateMagnetometer() below)
    mpu9250.setMagCalX(MagError[X], MagScale[X]);
    mpu9250.setMagCalY(MagError[Y], MagScale[Y]);
    mpu9250.setMagCalZ(MagError[Z], MagScale[Z]);

    mpu9250.setSrd(0); // sets both rate-gyro and accelerometer reads to 1 kHz, magnetometer reads to 100 Hz

  #endif
}


/* Read accelerometers and rate-gyros data from the IMU. 
   These values are scaled according to the IMU datasheet to put them into the correct units of gee's and degrees/s
   respectively.
   A simple first-order digital low-pass filter is used to get rid of high frequency noise in these raw signals.
   Generally you want to cut off everything past 80 Hz, but if your loop rate is not fast enough, the low pass filter
   will cause a lag in the readings. Finally, the constant errors found with subroutine calculate_IMU_error()
   are subtracted from the readings. */
void getIMUdata()
{
  int16_t Ac[3], Gy[3];

  /* Using the MPU 6050 IMU chip */
  #if defined USE_MPU6050_I2C

  // Pulls a sequence of bytes: 14 pairs of bytes (8-bit integers) are formed into the following two-byte values
  // (includes two bytes from the internal temperature sensor which we ignore).
  mpu6050.getMotion6(&Ac[NOSEWARDS], &Ac[PORTWARDS], &Ac[UPWARDS],
                     &Gy[STBD_WING_DROPPING], &Gy[NOSE_DROPPING], &Gy[HEADING_DECREASING]);

  /* Using the MPU 9250 IMU chip */
  #elif defined USE_MPU9250_SPI

    int16_t Mg[3];

    mpu9250.getMotion9(&Ac[NOSEWARDS], &Ac[PORTWARDS], &Ac[UPWARDS],
                       &Gy[STBD_WING_DROPPING], &Gy[NOSE_DROPPING], &Gy[HEADING_DECREASING])
                       &Mg[X], &Mg[Y], &Mg[Z]);
  #endif

  // Rescale the two-byte integers into proper real outputs:
  for (uint8_t ii = 0; ii < 3; ii++)
  {
    // Accelerometers, gees
    Accelerometer_gees[ii] = Ac[ii] / ACCEL_SCALE_FACTOR;

    // Correct the outputs with the calculated error values
    Accelerometer_gees[ii] -= Accelerometer_gees_Error[ii];

    // Low pass filter (1st-order lag) the accelerometers data
    low_pass_filter(&Accelerometer_gees[ii], &Accelerometer_gees_prev[ii], B_accel);
  

    // Rate gyros, degrees per second
    Rate_gyros_degs[ii] = Gy[ii] / GYRO_SCALE_FACTOR;

    // Correct the outputs with the calculated error values
    Rate_gyros_degs[ii] -= Rate_gyros_degs_Error[ii];

    // Low pass filter (1st-order lag) the rate gyros data
    low_pass_filter(&Rate_gyros_degs[ii], &Rate_gyros_degs_prev[ii], B_gyro);
  }


  /* Using the MPU 9250 IMU chip on-board magnetometers? */
  #if defined USE_MPU9250_SPI

  for (uint8_t ii = 0; ii < 3; ii++)
  {
    // Convert to microTesla and apply calculated error values
    Magnetometer[ii] = (Mg[ii] / 6.0f - MagError[ii]) * MagScale[ii];

    // Low pass filter (1st-order lag) the magnetometer data if required.
    // (unfiltered in Nicholas' code because B_mag was set to 1.0f)
    //low_pass_filter(&Magnetometer[ii], &Magnetometer_prev[ii], B_mag);
  }

  #endif

}


// Compute IMU accelerometer and rate gyro errors on startup by taking the average of several thousand readings
// N.B. IMU chip needs to be still, and at the correct attitude (engine/s off!).
void calculate_IMU_error(bool vertical)
{
  int16_t Ac[3], Gy[3];

  uint8_t ii;

  int c = 0;


  // Read IMU values 12000 times
  while (c < 12000)
  {
    #if defined USE_MPU6050_I2C

      // Get raw integer values from the IMU
      mpu6050.getMotion6(&Ac[NOSEWARDS], &Ac[PORTWARDS], &Ac[UPWARDS],
                         &Gy[STBD_WING_DROPPING], &Gy[NOSE_DROPPING], &Gy[HEADING_DECREASING]);

    #elif defined USE_MPU9250_SPI
      //mpu9250.getMotion9(&Ac[NOSEWARDS], &Ac[PORTWARDS], &Ac[UPWARDS],
      //                   &Gy[STBD_WING_DROPPING], &Gy[NOSE_DROPPING], &Gy[HEADING_DECREASING])
      //                   &Mg[X], &Mg[Y], &Mg[Z]);

      mpu9250.getMotion6(&Ac[NOSEWARDS], &Ac[PORTWARDS], &Ac[UPWARDS],
                         &Gy[STBD_WING_DROPPING], &Gy[NOSE_DROPPING], &Gy[HEADING_DECREASING]);

    #endif


    for (ii = 0; ii < 3; ii++)
    {
      // Rescale Accelerometer data into real values (gees)
      Accelerometer_gees[ii] = Ac[ii] / ACCEL_SCALE_FACTOR;

      // Sum all the readings
      Accelerometer_gees_Error[ii] += Accelerometer_gees[ii];

      // Rescale rate gyro data into real values (degrees per second)
      Rate_gyros_degs[ii] = Gy[ii] / GYRO_SCALE_FACTOR;
    
      // Sum all the readings
      Rate_gyros_degs_Error[ii] += Rate_gyros_degs[ii];
    }

    c++;
  }

  // Divide the sum by 12000 to get the averaged error values
  for (ii = 0; ii < 3; ii++)
  {
    Accelerometer_gees_Error[ii] /= c;
    Rate_gyros_degs_Error[ii]    /= c;
  }

  // The averaging process masks-out the gravity signal (this line adds it back in)
  // otherwise the sensor fusion won't be able to find it later.
  // Is the aircraft sitting vertically or horizontally at IMU power-up?
  if (vertical)
    Accelerometer_gees_Error[NOSEWARDS] -= 1.0f;
  else // horizontal
    Accelerometer_gees_Error[UPWARDS] -= 1.0f;

  // Print the error values...
  Serial.print("float AccError[0] = ");
  Serial.print(Accelerometer_gees_Error[NOSEWARDS]);
  Serial.println(";");
  Serial.print("float AccError[1] = ");
  Serial.print(Accelerometer_gees_Error[PORTWARDS]);
  Serial.println(";");
  Serial.print("float AccError[2] = ");
  Serial.print(Accelerometer_gees_Error[UPWARDS]);
  Serial.println(";");
  
  Serial.print("float GyroError[0] = ");
  Serial.print(Rate_gyros_degs_Error[STBD_WING_DROPPING]);
  Serial.println(";");
  Serial.print("float GyroError[1] = ");
  Serial.print(Rate_gyros_degs_Error[NOSE_DROPPING]);
  Serial.println(";");
  Serial.print("float GyroError[2] = ");
  Serial.print(Rate_gyros_degs_Error[HEADING_DECREASING]);
  Serial.println(";");

  /* ... so that you can paste them into these arrays at the top of this file (though NOT here!)
    float Accelerometer_gees_Error[3] = {0.0f, 0.0f, 0.0f};
    float Rate_gyros_degs_Error[3]    = {0.0f, 0.0f, 0.0f};
  */
}

void calibrateMagnetometer()
{
  #if defined USE_MPU9250_SPI 

    float success;

    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);

    // the sensor should be rotated in a figure 8 motion until complete
    Serial.println("Rotate the IMU about all axes in a figure 8 motion until complete.");
    Serial.println(" ");

    success = mpu9250.calibrateMag();

    if (success)
    {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");

      Serial.print("float MagError[0] = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagError[1] = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagError[2] = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");

      Serial.print("float MagScale[0] = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScale[1] = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScale[2] = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");

      /* ... so that you can paste them into these arrays at the top of this file (though NOT here!)
        float MagError[3] = {0.0f, 0.0f, 0.0f};
        float MagScale[3] = {1.0f, 1.0f, 1.0f};
      */

    }
    else
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
  
  #else
    Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer!");
  #endif

  while(1); // Halt code here so it won't enter the main loop until this function is commented out
}



/* Attitude estimation through sensor fusion.                       
   
   For example, see: https://en.wikipedia.org/wiki/Euler_angles 
   and https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  Aircraft rigid body-fixed inertial axes vectors:
  X = axial direction,  +ve nosewards through C.G.
  Y = side direction,   +ve to Port through C.G.
  Z = normal direction, +ve ceiling-wards through C.G.

  Roll  = rotation about the X axis, +ve = starboard wing falling.   
  Pitch = rotation about the Y axis, +ve = nose falling.               
  Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight.

  N.B! these are the opposite senses of standard aircraft Roll, Pitch, Yaw, angles! (Thank you Mr Madgwick).

  DirCos[][] is a rotation matrix (of direction cosines) describing the 3D orientation between
  the aircraft rigid-body axes and the following Earth inertial NWU axes:
  N = Northwards 
  W = Westwards  
  D = Upwards

  A Quaternion is a set of four numbers [quat[0], quat[1], quat[2], quat[3]] that also describes the 3D orientation
  between one set of axes (the aircraft axes here) and another (the Earth axes): you can always find
  one particular axis R that will allow you to rotate the aircraft axes so that they line-up with 
  the Earth axes.
  The Quaternian describes that axis R in 3D space (rx, ry, rz), and how much rotation (theta in radians) 
  you need about R to align the aircraft axes and Earth axes:
  quat[0] = cos(theta/2), quat[1] = rx * sin(theta/2), quat[2] = ry * sin(theta/2), quat[3] = rz * sin(theta/2)

  The IMU's gyros give angular rates around the aircraft body axes rather than the angles themselves,
  so we have to work with the rate of change of the Quaternion with time, and then time-integrate it to get the 
  actual current Quaternion.
  Once we have this, we can decompose the rotation about the Quaternian's single axis into the three more
  useful 'Euler' angles: Roll, Pitch, and Yaw.
   RMN */


/* Sebastian O. H. Madgwick's sensor fusion algorithm (Bristol University, U.K.) 
   This uses the IMU's accelerometers to sense where Earth's gravity is pointing (i.e. down!), then uses 
   this fixed reference to correct the drift of the axes caused by the inevitable rate gyros drifting.
   If magnetometer data is also available, a further correction for yaw drift is applied.
   See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms */
void Madgwicks_sensor_fusion(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float d_time)
{
  float bob[3], bx, bz;
  float d_quat_dt[4];
  float hx, hy;
  float recipNorm;
  float step[4], sq_accel, sq_mag;
  float q0q0, q1q1, q2q2, q3q3;

  uint8_t ii;


  // Convert rate gyroscopes degrees/sec to radians/sec
  gx *= DEG_TO_RADf; gy *= DEG_TO_RADf; gz *= DEG_TO_RADf;

  // Calculate the rate of change of the Quaternion from the rate gyroscope inputs
  d_quat_dt[0] = 0.5f * (-quat[1] * gx - quat[2] * gy - quat[3] * gz);
  d_quat_dt[1] = 0.5f * ( quat[0] * gx + quat[2] * gz - quat[3] * gy);
  d_quat_dt[2] = 0.5f * ( quat[0] * gy - quat[1] * gz + quat[3] * gx);
  d_quat_dt[3] = 0.5f * ( quat[0] * gz + quat[1] * gy - quat[2] * gx);


  // Compute feedback only if accelerometer measurements are valid:
  sq_accel = sq(ax) + sq(ay) + sq(az);

  // Less than X gee and accelerometer data isn't buggered?
  if ((sq_accel < max_gee_squared) && (sq_accel > 0.0f))
  {
    // Normalise the accelerometer inputs
    recipNorm = 1.0 / sqrtf(sq(ax) + sq(ay) + sq(az) );

    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    /* Madgwick's gradient descent algorithm's corrective step:
       Calculates where the Quaternian THINKS the Earth-axis 'Down' axis is, then compares this to where the
       IMU's accelerometers ACTUALLY detect where gravity 'down' currently is! Any mismatch between the two is
       converted into a Quaternion adjustment 'nudge' signal stored in array step[].
       (Zero mismatch gives zero values in step[])

       Note that this is the same formulae used in the original DrehmFlight, just reformalised, 
       and reduced by a factor of four (as it's getting normalised anyway). */

    bob[0] =        quat[1] * quat[3] - quat[0] * quat[2] - ax / 2.0f;
    bob[1] =        quat[0] * quat[1] + quat[2] * quat[3] - ay / 2.0f;
    bob[2] = 0.5f - quat[1] * quat[1] - quat[2] * quat[2] - az / 2.0f;

    // Calculate the required corrective 'nudge'
    step[0] = -quat[2] * bob[0] + quat[1] * bob[1];
    step[1] =  quat[3] * bob[0] + quat[0] * bob[1] - 2.0f * quat[1] * bob[2];
    step[2] = -quat[0] * bob[0] + quat[3] * bob[1] - 2.0f * quat[2] * bob[2];
    step[3] =  quat[1] * bob[0] + quat[2] * bob[1];


    /* Madgwick's gradient descent algorithm's additional corrective step for yaw, based on Earth's magnetic field:
       Calculates where the Quaternian THINKS the Earth-axis 'North' axis is, then compares this to where the
       magnetometers ACTUALLY detect where 'North' currently is! Any mismatch between the two is
       converted into a further Quaternion adjustment 'nudge' signal added to array step[].
       (Zero mismatch gives zero additional values in step[])

       Note that this is the same formulae used in the original DrehmFlight, just reformalised, 
       and again reduced by a factor of four (as it's getting normalised anyway). */

    sq_mag = sq(mx) + sq(my) + sq(mz);
   
    // Data isn't buggered? If it is, the following code gets skipped entirely
    if (sq_mag > 0.0f)
    {
      // Normalise the magnetometer inputs
      recipNorm = 1.0 / sqrtf(sq(mx) + sq(my) + sq(mz));

      mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      q0q0 = quat[0] * quat[0];
      q1q1 = quat[1] * quat[1];
      q2q2 = quat[2] * quat[2];
      q3q3 = quat[3] * quat[3];


      // Calculate the reference direction of Earth's magnetic field
      hx =        mx * (q0q0 + q1q1 - q2q2 - q3q3)
         + 2.0f * my * (quat[1] * quat[2] - quat[0] * quat[3])
         + 2.0f * mz * (quat[1] * quat[3] + quat[0] * quat[2]);

      hy = 2.0f * mx * (quat[0] * quat[3] + quat[1] * quat[2])
                + my * (q0q0 - q1q1 + q2q2 - q3q3)
         + 2.0f * mz * (quat[2] * quat[3] - quat[0] * quat[1]);
      

      bx = sqrtf(hx * hx + hy * hy) / 2.0f;

      bz = mx * (-quat[0] * quat[2] + quat[1] * quat[3])
         + my * ( quat[0] * quat[1] + quat[2] * quat[3])
         + mz * ( q0q0 - q1q1 - q2q2 + q3q3) / 2.0f;

      bob[0] = bx * (0.5f - q2q2 - q3q3)                    + bz * (quat[1] * quat[3] - quat[0] * quat[2]) - mx / 2.0f;
      bob[1] = bx * (quat[1] * quat[2] - quat[0] * quat[3]) + bz * (quat[0] * quat[1] + quat[2] * quat[3]) - my / 2.0f;
      bob[2] = bx * (quat[0] * quat[2] + quat[1] * quat[3]) + bz * (0.5f - q1q1 - q2q2)                    - mz / 2.0f;

      // Calculate the required additional corrective 'nudge'
      step[0] += (  bob[0] *  -bz * quat[2]
                  + bob[1] * (-bx * quat[3] + bz * quat[1])
                  + bob[2] *   bx * quat[2]);

      step[1] += (  bob[0] *  bz * quat[3]
                  + bob[1] * (bx * quat[2] +        bz * quat[0])
                  + bob[2] * (bx * quat[3] - 2.0f * bz * quat[1]) );

      step[2] += (  bob[0] * (-2.0f * bx * quat[2] -        bz * quat[0])
                  + bob[1] * (        bx * quat[1] +        bz * quat[3])
                  + bob[2] * (        bx * quat[0] - 2.0f * bz * quat[2]) );

      step[3] += (  bob[0] * (-2.0f * bx * quat[3] + bz * quat[1])
                  + bob[1] * (       -bx * quat[0] + bz * quat[2])
                  + bob[2] *          bx * quat[1]);

    }


    // Normalise the step magnitude vector step[]
    recipNorm = 1.0f / sqrtf( sq(step[0]) + sq(step[1]) + sq(step[2]) + sq(step[3]) ); 

    for (ii = 0; ii < 4; ii++)
    {
      step[ii] *= recipNorm;

      // Nudge the Quaternion back into alignment. Constant 'B_madgwick' determines the strength of the 'nudge'.
      d_quat_dt[ii] -= B_madgwick * step[ii];
    }

  }

  // Time-integrate the rate of change of the Quaternion to yield the current Quaternion
  for (ii = 0; ii < 4; ii++)
    quat[ii] += d_quat_dt[ii] * d_time;

  // Normalise the Quaternion
  recipNorm = 1.0f / sqrtf( sq(quat[0]) + sq(quat[1]) + sq(quat[2]) + sq(quat[3]) );

  for (ii = 0; ii < 4; ii++)
    quat[ii] *= recipNorm;
}


/* And finally, decompose the Quaternion into standard Euler axes using a Direction cosine rotation matrix. */
void Euler(bool add_ninety)
{
  float DirCos[3][3]; // Direction cosine matrix

  /* Compute the Direction Cosine matrix from the Quaternion.
     Direction Cosines convert from the body axes of the aircraft to Earth axes (or vice-versa).
     (Could use the inhomogeneous expression for speed because we've already normalised the Quaternion:
     https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles )
     Rick Newlands */

  float sq_quat[4];
  float quat_01, quat_02, quat_03, quat_12, quat_13, quat_23;


  sq_quat[1] = sq(quat[1]);
  sq_quat[2] = sq(quat[2]);
  sq_quat[3] = sq(quat[3]);

  quat_01 = quat[0] * quat[1];
  quat_02 = quat[0] * quat[2];
  quat_03 = quat[0] * quat[3];
  quat_12 = quat[1] * quat[2];
  quat_13 = quat[1] * quat[3];
  quat_23 = quat[2] * quat[3];


  DirCos[0][0] = 1.0f - 2.0f * (sq_quat[2] + sq_quat[3]); // sq(quat[0]) + sq(quat[1]) - sq(quat[2]) - sq(quat[3]);
  DirCos[0][1] = 2.0f * (quat_12 + quat_03);
  DirCos[0][2] = 2.0f * (quat_13 - quat_02);
  DirCos[1][0] = 2.0f * (quat_12 - quat_03);
  DirCos[1][1] = 1.0f - 2.0f * (sq_quat[1] + sq_quat[3]); // sq(quat[0]) - sq(quat[1]) + sq(quat[2]) - sq(quat[3]);
  DirCos[1][2] = 2.0f * (quat_01 + quat_23);
  DirCos[2][0] = 2.0f * (quat_02 + quat_13);
  DirCos[2][1] = 2.0f * (quat_23 - quat_01);
  DirCos[2][2] = 1.0f - 2.0f * (sq_quat[1] + sq_quat[2]); // sq(quat[0]) - sq(quat[1]) - sq(quat[2]) + sq(quat[3]);


  /* Compute Euler angles from Direction cosines:
     Note that we're using North-West-Up (NWU) axes rather than North-East-Down (NED) axes, hence negation for pitch angle.

     
     Roll  = rotation about the X axis, +ve = starboard wing falling.   
     Pitch = rotation about the Y axis, +ve = nose falling.               
     Yaw   = rotation about the Z axis, +ve = compass heading decreasing when in straight and level flight.

     N.B: 'Yaw' is actually True heading, but relative to an arbitrary zero unless pre-set, and 'roll' is actually bank angle
          using the horizontal plane as a reference.
     N.B: These are the directions the aircraft is pointing, not necessarily the directions it's moving!

     Standard function atan2() has inbuilt divide-by-zero protection, and returns an angle between -PI and PI radians.
     Using a safer version of the function to return the arc sin() of an angle.
     Rick Newlands */

  // std aircraft (Tait-Bryan angles): psi (heading), theta (pitch), phi (roll) or z-y′-x″ (3-2-1)
  horizontal_IMU.roll_degs  =  atan2f(DirCos[1][2], DirCos[2][2]) * RAD_TO_DEGf;
  horizontal_IMU.pitch_degs = -Safe_arcsinf(-DirCos[0][2])        * RAD_TO_DEGf; // DirCos02 negated for reverse pitch definition Rick N
  horizontal_IMU.yaw_degs   = -atan2f(DirCos[0][1], DirCos[0][0]) * RAD_TO_DEGf;


  /* Tailsitter mode: when an aircraft is oriented at 90 degrees of pitch, the gyro/accelerometer IMU chip is also now
     in a vertical orientation.
     This causes a problem known as 'gimbal lock' from the days when the gyros were mechanical, and their gimbals
     lining-up at 90 degrees of pitch would literally topple the spinning gyro wheel inside (see the movie 'Apollo 13').
     the Euler angle calculations have exactly the same 'gimbal lock' problem (albeit mathematically: tan(90) is undefined)
     and fall apart - garbage values are output. See https://www.youtube.com/watch?v=zc8b2Jo7mno for details.
     So we need to re-compute the Euler angles in a separate set of axes set at 90 degrees of pitch to the usual ones.
     Note that we're now using Up-West-South (UWS) axes rather than North-East-Down (NED) axes. Rick Newlands

     The whole point of the mathematical invention of the 'Quaternion' is that it NEVER gimbal locks (so decoding it
     into Euler angles that do is a bit of an own-goal ;-) Computer graphics routines stick with the Quaternion alone. 
     
     'Yaw' is now a sort of Declination, but relative to the local horizon 
     
     N.B: this new set of roll, pitch, yaw values will match the 'horizontal' values BUT ONLY AT ZERO ROLL ANGLE.
     Fortunately, we'd normally select for zero 'roll' on autopilot during a steep climb.
     */
  // reverse phi,theta rotation order for tailsitter in hover: psi (heading), phi (roll), theta (pitch)  or z-x′-y″ (3-1-2)
  vertical_IMU.roll_degs  =  Safe_arcsinf(DirCos[1][2])         * RAD_TO_DEGf;
  vertical_IMU.pitch_degs =   atan2(DirCos[0][2], DirCos[2][2]) * RAD_TO_DEGf; 
  vertical_IMU.yaw_degs   = -atan2(-DirCos[1][0], DirCos[1][1]) * RAD_TO_DEGf;

  if (add_ninety == true)
    vertical_IMU.pitch_degs += 90.0f; // zero degrees is now nose-vertical!
  

  /* Calculate the vertical acceleration of the vehicle in the Madgwick-corrected 'up' direction.
     Using direction cosines to avoid gimbal lock in vertical ascents. */
  upward_accel_metres_per_sec2 = (  Accelerometer_gees[NOSEWARDS] * DirCos[0][2] 
                                  + Accelerometer_gees[PORTWARDS] * DirCos[1][2]
                                  + Accelerometer_gees[UPWARDS]   * DirCos[2][2]) * -G_METRICf;
}


/* Low pass filter (weighting must be equal to or less than 1.0f) 
   Writes the *values back into the 'current' and 'previous' values Rick N */
void low_pass_filter(float *current, float *previous, float weighting)
{
  *current = weighting * *current + (1.0f - weighting) * *previous;

  // update last-times value
  *previous = *current;
}


/* Enhanced arcsin function that deals with right angles (Pi/2 radians) */
float Safe_arcsinf(float value)
{
  if (value < -1.0f)     return(-HALF_PIf);
  else if (value > 1.0f) return( HALF_PIf);
  else return(asinf(value));
}


/* Trapezoidal integration subroutine */
float Integrate(float older, float newer, float interval)
{
  return(0.5f * (older + newer) * interval);
};


/* Basic estimation of vertical parameters.
   N.B! No Coriolis corrections (no 'centrifugal force' corrections based on inertial flight speed in a Loop or a tight turn)
   cos we don't know what the value of the inertial flightspeed is: flightspeed vector minus the wind vector.
   RMN May 2026 */
void Vertical_parameters(float d_time)
{
  // Integration of upward acceleration to get rate of climb (metres per second)
  upward_velocity_metres_per_sec += Integrate(old_upward_accel_metres_per_sec2, upward_accel_metres_per_sec2, d_time);

  // Integration of upward velocity to get height esimate (metres above launch altitude)
  height_metres += Integrate(old_upward_velocity_metres_per_sec, upward_velocity_metres_per_sec, d_time);

  altitude_AMSL = launch_altitude_metres + height_metres;

  // Save last-times values
  old_upward_accel_metres_per_sec2   = upward_accel_metres_per_sec2;
  old_upward_velocity_metres_per_sec = upward_velocity_metres_per_sec;
}