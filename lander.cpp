// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  static double prev_r;
  double target_altitude;
  if(int(simulation_time) % 240 < 60){
    target_altitude = 500;
  }
  else if(int(simulation_time) % 240 < 120) {
    target_altitude = 1000;
  }
  else if(int(simulation_time) % 240 < 180) {
    target_altitude = 500;
  }
  else{
    target_altitude = 1;
  }
  double f_eq = GRAVITY * MARS_MASS * lander_mass() / pow((MARS_RADIUS + target_altitude), 2);
  double KP = -0.0045;
  double KD = -0.052;
  double descent_rate = velocity * position;
  double altitude = position.abs() - MARS_RADIUS;
  double r = altitude - target_altitude;
  double dr = r - prev_r;
  prev_r = r;
  double f = KP * r + KD * dr/delta_t;
  throttle = (f_eq) / MAX_THRUST + f;
}

float lander_mass(void)
{
  return FUEL_CAPACITY * fuel * FUEL_DENSITY + UNLOADED_LANDER_MASS;
}

vector3d gravity_wrt_world(void)
{
  vector3d a = -(GRAVITY * MARS_MASS * position.norm()) / pow(position.abs(), 2);
  return a;
}

vector3d drag_wrt_world(void)
{
  float area = 0.5 * M_PI * pow(LANDER_SIZE, 2);
  vector3d f = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * area * velocity * velocity.abs();
  if(parachute_status == DEPLOYED){
    area = 5 * pow(2 * LANDER_SIZE, 2);
    f += -0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * area * velocity * velocity.abs();
  }
  return f / lander_mass();
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{

  x_list.push_back(position);

  vector3d a = (thrust_wrt_world() / lander_mass()) + gravity_wrt_world() + drag_wrt_world();

  position = (2 * position) - x_list.end()[-2] + (delta_t * delta_t * a);
  velocity = (position - x_list.end()[-1]) / delta_t;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "500m static";
  scenario_description[7] = "510m static";
  scenario_description[8] = "700m descent";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 6:
    // a descent from rest at 500m altitude
    position = vector3d(0.0, -(MARS_RADIUS + 500.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 7:
    // a descent from rest at 510m altitude
    position = vector3d(0.0, -(MARS_RADIUS + 510.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 8:
    // a descent from rest at 700 altitude
    position = vector3d(0.0, -(MARS_RADIUS + 700.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 9:
    // a descent from rest at 1 altitude
    position = vector3d(0.0, -(MARS_RADIUS + 1.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  }
  x_list.push_back(position - delta_t * velocity);
}
