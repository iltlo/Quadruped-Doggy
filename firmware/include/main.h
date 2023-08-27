#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_PWMServoDriver.h>
#include "datatypes.h"


/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function
const float frequency = 440.0; // ## Hz

/// Kinematics Parameters

//: stores the location, rotation and scale of the main [body]
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},   // ## {deg, deg, deg}
  {300, 40, 180} // ## {mm, mm, mm}
};

//: stores the parent joint location relative to the [body]
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { -50, 0, 0}  // ## {mm, mm, mm}
};
const float bone_length = 105; // ## mm

//: high level parameters for the step function
const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
extern float vrt_offset; // ## mm
extern float hrz_offset; // ## mm

extern float base_offset[];
const float precision = 0.001; // ## mm

extern int state; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection

//: this is an interpolation function used to smooth
float inter(float in, float en, float pl);

void commands_exe(float val1, float val2, float val3);

// !! make sure you have enabled Newline or Carriage return
void handle_serial();

#endif // MAIN_H
