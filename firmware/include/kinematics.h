#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

#include "main.h"

/*
  ==============================
  KINEMATICS PARAMETERS
  ==============================
*/

//: this array stores the inverse direction(relative to the body) of each parent joint.
const float l_inv[4][2] = {
  { +1.f, -1.f}, // ## {dir, dir}
  { -1.f, -1.f}, // ## {dir, dir}
  { -1.f, +1.f}, // ## {dir, dir}
  { +1.f, +1.f}  // ## {dir, dir}
};

/*
  ::: HANDLE LOOP :::
*/
void handle_kinematics(datatypes::Vector2D _dir, float _turn, float _height, float period);

/*
  ::: TRIGONOMETRIC FUNCTIONS :::
*/
//: base calculation function
inline float c_base(float angle1) {
  return sin(radians(angle1 / 2)) * bone_length * 2;
}

//: final triangle calculation function
datatypes::Rotator c_triangle(float a0, float a1, float b0);

#endif // KINEMATICS_H