/***************************************************************************
  This is a library for the Earth's Atmosphere as sensed by:
  
  BMP280 temperature & pressure sensor

  Rick Newlands, Professional Flight simulation Engineer
  2026

 ***************************************************************************/

#include <Arduino.h>          // Required for Visual Studio Code, and standard type defines
#include <Wire.h>              // I2C
#include <Adafruit_Sensor.h>    // Adafruit interfacing
#include <Adafruit_BMP280.h>     // Adafruit interfacing
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands


// Declare an instance of Adafruit_BMP280 class
Adafruit_BMP280 bmp;

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

float Altitude_metres;
float Density, Density_ratio;
float height_offset_metres;
float ISA_offset_P, ISA_offset_T;
float Pressure_Pa;
float Temperature_C;



/* Functions and equations for the Troposphere (0 to 11 Km altitude) assuming the International Standard Atmosphere (ISA):

   h = height (in metres) 
   P = measured atmospheric pressure at altitude
   T = measured atmospheric pressure at altitude

   P0 = current sea-level pressure.    (If unknown just use the standard ISA value)
   T0 = current sea-level temperature. (If unknown just use the standard ISA value)

   ISA Standard Troposphere lapse rate = -0.0065 Kelvin per metre (up to 11 Km)

   Temperature relation using this lapse rate:  T = T0 - 0.0065 * h   or:  (T/T0) = 1.0 - 0.0065 * h

   Pressure relation based on this temperature relation: (P/P0) = (T/T0)^-5.2558797

   Rearranging:

       ((P0/P)^(1.0/5.2558797) - 1.0f) * T0  
   h = ------------------------------------
                   -0.0065
*/

#define ISA_T0          288.15f // ISA standard sea level temperature degrees Kelvin (= 15.0 degrees Celcius)
#define ISA_P0       101325.0f   // ISA standard sea level pressure Pascals (N/m^2)
#define ISA_D0            1.225f  // ISA standard sea level density kg/m^3
#define R_air           287.052   // specific gas constant for air
#define Tropo_lapse_rate -0.0065   // ISA standard dT/dh  (degree Kelvin per metre)
#define Tropo_exponent    5.2558797 // ISA standard pressure equation exponent
#define Reciprocal_exponent (1.0f / Tropo_exponent) // reciprical


/* Estimate sea-level conditions from current measurements recorded at your flying site.
   Rick N, May 2026 */
void Estimate_seaLevel_conditions(float site_pressure_Pa, float site_temperature_C, float site_elevation_m)
{
  float site_temperature_K = site_temperature_C + CENTIGRADE_TO_KELVINf; // convert to Kelvin

  float seaLevel_temperature_K = site_temperature_K - Tropo_lapse_rate * site_elevation_m;

  float seaLevel_pressure_Pa = site_pressure_Pa / pow((site_temperature_K / seaLevel_temperature_K), Tropo_exponent);

  //Serial.print("SeaLevel T0, P0: ");
  //Serial.print(seaLevel_temperature_K); Serial.print(" °K "); Serial.print(seaLevel_pressure_Pa); Serial.println(" Pa");


  // Calculate offsets from standard ISA sea-level conditions:
  ISA_offset_T = seaLevel_temperature_K - ISA_T0;
  ISA_offset_P = seaLevel_pressure_Pa   - ISA_P0;

  Serial.print("ISA offsets T0, P0: ");
  Serial.print(ISA_offset_T); Serial.print(" °C "); Serial.print(ISA_offset_P); Serial.println(" Pa");
}


/* Calculate the altitude (in metres) from a recorded Atmospheric pressure and known sea-level conditions.
   Rick N, May 2026 */
float Calculate_altitude(float pressure_at_altitude_Pa)
{
  float pressure_ratio = pressure_at_altitude_Pa / (ISA_P0 + ISA_offset_P);

  float altitude_in_metres = (ISA_T0 + ISA_offset_T) * (pow(pressure_ratio, Reciprocal_exponent) - 1.0f) / Tropo_lapse_rate;

  return(altitude_in_metres);
}


void Setup_BMP()
{
  delay(100); // Gives time for I2C to initialise

  Serial.println("Setting up the BMP 280");

  bool status = bmp.begin(0x76);  // change to 0x77 if your board uses that address

  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring or address!");

    Serial.print("SensorID: 0x"); Serial.println(bmp.sensorID(), 16);
    while (1)
    {
      delay(1000);
    }
  }

  Serial.println("BMP280 initialised successfully.");


  /* Normal mode, 1x temperature, 4x pressure, filter 8x, 1ms standby period */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,   // temperature
                  Adafruit_BMP280::SAMPLING_X4,   // pressure 
                  Adafruit_BMP280::FILTER_X8,     // (4 or 8 for rocketry, 16 for drone)
                  Adafruit_BMP280::STANDBY_MS_1 );

  Serial.println("BMP 280 setup successful!");

  // Reset
  ISA_offset_P = ISA_offset_T = 0.0f;

  Temperature_C = bmp.readTemperature();
  Pressure_Pa   = bmp.readPressure();

  // Calculate offsets from ISA sea-level conditions
  Estimate_seaLevel_conditions(Pressure_Pa, Temperature_C, 22.0f);
}


void Read_BMP_values()
{
  Temperature_C = bmp.readTemperature();
  Pressure_Pa   = bmp.readPressure();

  Density = Pressure_Pa / (R_air * Temperature_C); // Ideal gas law, kg/m3

  Density_ratio = Density / ISA_D0;

  Altitude_metres = Calculate_altitude(Pressure_Pa);
}
