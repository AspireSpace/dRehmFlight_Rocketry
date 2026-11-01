/* Deal with mass changes due to diminishing rocket propellant.

When rocket propellant burns, it ejects itself out the rocket nozzle. This causes balance issues!
As the propellant mass diminishes, the overall aircraft CG shifts (typically forwards) which alters the distance
between the CG and all of the aerodynamic flight control surfaces (elevons, canards etc).
This change of the aerodynamic control surface 'lever arms' changes their effectiveness: their 'gains' change,
so the controlling PID gains need to change to compensate.
Changes between the CG distances to each propeller on a quadcopter or tricopter drone also need dealing with.

Furthermore, the moments of inertia of the rocket propellant isn't trivial compared to the aircraft, so as they 
change, the aircraft interias also change, and the PIDS need to know this as well.

This code compensates for the propellant mass changes over the rocket burn to automatically adjust the flight
control PID gains so you don't have to.
It takes your burnout (propellant-empty) flight control PID values, and tweaks them for the presence of
the propellant.

You will have to measure some inertia values on your aircraft (see:   )

Rick Newlands June 2026, taken from my standard commercial flight simulator code models.
*/


//========================================================================================================================//


// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Arduino.h>
#include "dRehmFlight_Rocketry.h" // Project header file Rick Newlands



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

/* Local variables */
float booster_motor_x_moment, booster_motor_y_moment, booster_motor_z_moment;
float booster_zero_fuel_mass;
float booster_zero_fuel_x_moment, booster_zero_fuel_y_moment, booster_zero_fuel_z_moment;
float zero_fuel_x_moment, zero_fuel_y_moment, zero_fuel_z_moment;


// Locals
void init_mass_properties();
void simulate_mass_properties();


/* 6-DOF mass properties datum axes:                          
   x= +ve rearwards from nosetip (opposite sense to flightmodel X axis)
   y= +ve to Starboard from centreline                                 
   z= +ve downwards below fuselage  */


/* Generic mass-moment calculation subroutine */
void moment(float mmass,
            float mcgx, float mcgy, float mcgz,
            float *xx_moment, float *yy_moment, float *zz_moment)
{
  *xx_moment = mmass * mcgx;
  *yy_moment = mmass * mcgy;
  *zz_moment = mmass * mcgz;
}

/* Generic Moments and Products of inertias handling subroutine.
   Also used for parallel axis theorem transformations. 

   Moments of inertia, and Products of inertia
   (Iyz = Ixy = 0 unless unevenly loaded)     
   Ixx = inertia about roll axis              
   Iyy = inertia about pitch axis             
   Izz = inertia about yaw axis               
   Ixz = cross-product about x and z axes     
   Ixy = cross-product about x and y axes     
   Iyz = cross-product about y and z axes */
void inertia(float imass,
             float icgx,  float icgy,  float icgz,
             float *iixx, float *iiyy, float *iizz,
             float *iixz, float *iixy, float *iiyz)
{
  float sqr_cgx, sqr_cgy, sqr_cgz;

  sqr_cgx = sq(icgx); sqr_cgy = sq(icgy); sqr_cgz = sq(icgz);
  
  // Moments of inertia
  *iixx += imass * (sqr_cgy + sqr_cgz);
  *iiyy += imass * (sqr_cgx + sqr_cgz);
  *iizz += imass * (sqr_cgx + sqr_cgy);

  // Products of inertia
  *iixz += imass * icgx * icgz;
  *iixy += imass * icgx * icgy;
  *iiyz += imass * icgy * icgz;
}



void init_mass_properties()
{
  solid_x_moment         = solid_y_moment         = solid_z_moment       = 0.0f;
  solid_motor_x_moment   = solid_motor_y_moment   = solid_motor_z_moment = 0.0f;

  zero_fuel_x_moment = zero_fuel_mass * zero_fuel_cgx; // kg-metres x
  zero_fuel_y_moment = zero_fuel_mass * zero_fuel_cgy; // kg-metres y
  zero_fuel_z_moment = zero_fuel_mass * zero_fuel_cgz; // kg-metres z
}

void simulate_mass_properties()
{
 float booster_grain_mass;
 float cgx, cgy, cgz;
 float gross_mass;
 float mass_in_chamber;
 float overall_x_moment, overall_y_moment, overall_z_moment;
 float solid_motor_x_moment, solid_motor_y_moment, solid_motor_z_moment;
 float sqr_cgx, sqr_cgy, sqr_cgz;


 /************* solid motor *******************/
 /* Needs to be updated constantly as solid propellant is used */


   /* Reference booster zero-fuel hardware to x(0), y(0), z(0) (nosetip) */
   cgx = booster.CG_x;
   cgy = 0.0; // assuming symmetrically arranged boosters
   cgz = 0.0; // assuming symmetrically arranged boosters

   booster_zero_fuel_mass = booster.zero_fuel_mass * booster.number_of_motors;

   moment(booster_zero_fuel_mass,
          cgx, cgy, cgz,
          &booster_zero_fuel_x_moment, &booster_zero_fuel_y_moment, &booster_zero_fuel_z_moment);

   // Already multiplied by number of motors   booster.hot_gas_mass = 0.0 for now
   mass_in_chamber = hot_gas_mass + fuel_grain_mass + motor_consumables_mass;

   /* Reference solid propellant and gas moments to x(0), y(0), z(0) (nosetip) */
   cgx = booster.solid_CG_x + booster.fuel_grain_position_x;
   cgy = 0.0;
   cgz = 0.0;

   moment(mass_in_chamber,
          cgx, cgy, cgz,
          &booster_motor_x_moment, &booster_motor_y_moment, &booster_motor_z_moment);


   /* Zero-fuel fuel hardware inertias are referenced to their local C.G.
      Re-reference to x(0), y(0), z(0) using parallel axes theorem:    
      I(about ref.point) = I(about C.G.) + ((dist between ref.point and the C.G.)^2) * reference mass    

      Write the new values back into the solid fuel inertias */

   cgx = booster.CG_x; // firstly, just relocate the x-coordinate
   cgy = 0.0;
   cgz = 0.0;

   inertia(booster_zero_fuel_mass, // already multiplied by number of motors
           cgx, cgy, cgz,
           &booster.zero_fuel_ixx, &booster.zero_fuel_iyy, &booster.zero_fuel_izz,
           &booster.zero_fuel_ixz, &booster.zero_fuel_ixy, &booster.zero_fuel_iyz);


   // Now move the two boosters on the X-Y plane laterally off the roll axis.
   sqr_cgx = 0.0;
   sqr_cgy = booster.fuel_grain_position_y * booster.fuel_grain_position_y;
   sqr_cgz = 0.0;
 
   // moments of inertia (products are zero)
   booster.zero_fuel_ixx += 2.0 * booster.zero_fuel_mass * (sqr_cgy + sqr_cgz); // roll
   booster.zero_fuel_izz += 2.0 * booster.zero_fuel_mass * (sqr_cgx + sqr_cgy); // yaw
  
 

   /* Solid fuel inertias are referenced to their local C.G.           
      Re-reference to x(0), y(0), z(0) using parallel axes theorem:    
      I(about ref.point) = I(about C.G.) + ((dist between ref.point and C.G.)^2) * reference mass

      Write the new values back into the solid fuel inertias.

      Re-reference fuel inertias using parallel axes theorem:          
      I(about ref.point) = I(about C.G.) + ((dist between ref.point and C.G.)^2) * reference mass */

   cgx = booster.solid_CG_x + booster.fuel_grain_position_x; // firstly, just relocate the x-coordinate
   cgy = 0.0;
   cgz = 0.0;

   inertia(mass_in_chamber, // already multiplied by number of motors
           cgx, cgy, cgz,
           &booster.solid_ixx, &booster.solid_iyy, &booster.solid_izz,
           &booster.solid_ixz, &booster.solid_ixy, &booster.solid_iyz);


   // Now move the boosters laterally off the roll axis, starting with the two boosters on the X-Y plane
   sqr_cgx = 0.0;
   sqr_cgy = booster.fuel_grain_position_y * booster.fuel_grain_position_y;
   sqr_cgz = 0.0;
 
   // Per booster
   booster_grain_mass = booster.fuel_grain_mass / booster.number_of_motors;

   // Moments of inertia (products are zero)
   booster.solid_ixx += 2.0 * booster_grain_mass * (sqr_cgy + sqr_cgz); // roll
   booster.solid_izz += 2.0 * booster_grain_mass * (sqr_cgx + sqr_cgy); // yaw
  





 /************* Overall Mass (kg) *****************/
 
 total_propellant_mass = solid.hot_gas_mass + solid.fuel_grain_mass   
  
 gross_mass = total_propellant_mass + zero_fuel_mass
            + booster_zero_fuel_mass + booster.motor_consumables_mass + booster.motor_casing_mass;

 // output to flightmodel
 mass   = gross_mass;
 weight = gross_mass * G_METRIC;



 /* Overall Centre of Gravity (metres) */

 overall_x_moment = zero_fuel_x_moment + booster_zero_fuel_x_moment
     + solid_motor_x_moment + booster_motor_x_moment;

 overall_y_moment = zero_fuel_y_moment + booster_zero_fuel_y_moment
     + solid_motor_y_moment + booster_motor_y_moment;

 overall_z_moment = zero_fuel_z_moment + booster_zero_fuel_z_moment
     + solid_motor_z_moment + booster_motor_z_moment;


 CG_x = overall_x_moment / gross_mass; // +ve rearwards from datum (nosetip)
 CG_y = overall_y_moment / gross_mass; // +ve to Starboard from datum (centreline axis)
 CG_z = overall_z_moment / gross_mass; // +ve floorwards from datum (centreline axis)

 // For debugging
 if (total_propellant_mass > 0.0)
   total_propellant_cgx
     = (solid_motor_x_moment + booster_motor_x_moment) / total_propellant_mass;

 
 /* Overall Moments/products of inertia (kgm2) */

 Ixx = zero_fuel_ixx     + booster.zero_fuel_ixx
     + solid.solid_ixx   + solid.gas_ixx;

 Iyy = zero_fuel_iyy     + booster.zero_fuel_iyy
     + solid.solid_iyy   + solid.gas_iyy;

 Izz = zero_fuel_izz     + booster.zero_fuel_izz
     + solid.solid_izz   + solid.gas_izz;


 Ixz = zero_fuel_ixz     + booster.zero_fuel_ixz
     + solid.solid_ixz   + solid.gas_ixz;


 Ixy = zero_fuel_ixy     + booster.zero_fuel_ixy
     + solid.solid_ixy   + solid.gas_ixy;


 Iyz = zero_fuel_iyz     + booster.zero_fuel_iyz
     + solid.solid_iyz   + solid.gas_iyz;



 /* The inertias are currently referenced to x(0), y(0), y(0)

    So re-reference the inertias to the current C.G. using the parallel axes theorem:   
    I(about ref.point) = I(about C.G.) + ((dist between ref.point and C.G.)**2) * reference mass        
    Note the -ve term, (-ve distance, not mass) as theorem is operating 'in reverse' FROM ref. point TO C.G.,
    which will reduce the inertia values as required.

    Re-reference the inertias, and write new values back into the inertias. */
 inertia((-gross_mass),   // -ve because this calculation is 'in reverse'
         CG_x, CG_y, CG_z,
         &Ixx, &Iyy, &Izz,
         &Ixz, &Ixy, &Iyz);
}