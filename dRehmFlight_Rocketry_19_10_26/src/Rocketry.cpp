/* Rocketry flight computer simulator for purely vertical ascents under rocket power.
   Based on the venerable old European Rocket Data Aquisition System ('RDAS')
  
   Rick Newlands

   21/08/2025 Initial release
*/

// Required libraries
#include <Arduino.h>             // Required for Visual Studio Code
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands

#define trigger_time 0.25f // RDAS


bool apogee_detect;
bool burnout;
bool launch_detect;
bool post_boost_unfreeze;

float Apogee_detect_time, Apogee_sample_time, Ascent_time;
float Boost_unfreeze_time, Burn_time;

extern float upward_accel_metres_per_sec2, upward_velocity_metres_per_sec;


void Initialise_Rocketry()
{
  Ascent_time = Apogee_sample_time = 0.0f;

  Apogee_detect_time = Burn_time + 2.0f; // wait X seconds after motor burnout before checking for apogee

  apogee_detect = launch_detect = burnout = post_boost_unfreeze = false; // reset
}


/* RDAS-ish flight computer simulation */
void Rocketry_Computer(float d_time)
{
  // RDAS launch-trigger logic:
  // Have 2.5 vertical gees been continually detected for over 0.25 seconds?
  if ((upward_accel_metres_per_sec2 > (2.5f * G_METRICf)) || (launch_detect))
  {
    if ((Ascent_time > trigger_time) && (launch_detect == false))
      launch_detect = true; // latch-on


    // Don't try any thrust-vectoring until motor boost-thrusts have finished!
    if (Ascent_time > Boost_unfreeze_time)
      post_boost_unfreeze = true;


    // Post-burnout
    if (Ascent_time > Burn_time)
    {
      burnout             = true;
      post_boost_unfreeze = false;
    }
  

    // Apogee detection (for parachute deployment if required): is airspeed increasing just after apogee?
    if ((Ascent_time > Apogee_detect_time) && (apogee_detect == false))
    {
      if (upward_velocity_metres_per_sec < 0.0f)   // post-burnout deceleration due to Drag
        Apogee_sample_time = 0.0f; // so keep resetting timer
      else if (Apogee_sample_time > trigger_time)
        apogee_detect = true;

      Apogee_sample_time += d_time; // update timer
    }
    

    Ascent_time += d_time; // update rocket ascent mission time
  }
  else // reset mission timer
    Ascent_time = 0.0f;
}

