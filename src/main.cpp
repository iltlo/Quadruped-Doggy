/*
  Quadruped robot arduino sketch.
  Reference: https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/

#include <Arduino.h>
#include "main.h"
#include "kinematics.h"
#include "hardware.h"
#include "datatypes.h"


/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//: high level parameters for the step function
float vrt_offset = - 16.50; // ## mm
float hrz_offset = - 6.00; // ## mm
float base_offset[] = { 0, -1, 0, -2};

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> indicates the direction of rotation
float height = 0; //> indicates the leg extension

int state = 0; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 10.0; //> indicates the number of steps every second

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body

unsigned long duration;
int sample_sum, sample_num = 10,
                sample_index;
float freq;

// #define _havePS4
#ifndef _havePS4
#include <PS4Controller.h>

float vo, ho;
void init_input() {
  PS4.begin("F8:C3:9E:3F:F8:10"); // !! replace with your own DualShock4 Controller Bluetooth MAC address
  vo = vrt_offset;
  ho = hrz_offset;
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;
void handle_input() {
  if (PS4.isConnected()) {
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone
      height = y1;
    } else height = 0;
  }

  if (PS4.data.button.touchpad) { //> checks the touchpad state
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;
    }
  } else _tb = true;
}
#endif // _havePS4


//: this is an interpolation function used to smooth
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  //: properties 0 is used to calibrate the joints
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  //: properties 1 is used for small adjustments to balance the weight
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}

// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing, (1) uses serial as input
void handle_serial() {
  //: reads and stores the serial data
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

void setup() {
  Serial.begin(115200);

  init_hardware();
  // init_input(); // !! comment this line if you don't have a PS4 controller
}

void loop() {
  duration = millis();

  handle_hardware();
  handle_kinematics(_direction, turn, height, _period);

  // handle_input();  // !! comment this line if you don't have a PS4 controller

  if (Serial.available())
    handle_serial();

  // this code gets the frequency of the loop function
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Serial.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
}