#ifndef __HARDWARE__
#define __HARDWARE__
#include "Config.hpp"
#include "GPIOservo.hpp"
#include "datatypes.h"
/*
  #include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
*/

class Hardware {
 private:
  /*
    ==============================
    HARDWARE - SERVO PARAMETERS
    ==============================
  */
#ifdef __IO_BOARD_HERKULES__
  GPIOservo s_output[4][3] = {
      {GPIOservo(25), GPIOservo(33),
       GPIOservo(32)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // right back
                        //
      {GPIOservo(12), GPIOservo(14),
       GPIOservo(27)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // right front
                        //
      {GPIOservo(15), GPIOservo(2),
       GPIOservo(4)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                       // left front
                       //
      {GPIOservo(5), GPIOservo(18),
       GPIOservo(19)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // left back
  };
#else
  GPIOservo s_output[4][3] = {
      {GPIOservo(4), GPIOservo(2),
       GPIOservo(15)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // right back
                        //
      {GPIOservo(19), GPIOservo(18),
       GPIOservo(5)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                       // right front
                       //
      {GPIOservo(32), GPIOservo(33),
       GPIOservo(25)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // left front
                        //
      {GPIOservo(27), GPIOservo(14),
       GPIOservo(12)},  // ## {shoulder chnl, upper chnl, lower chnl} robot's
                        // left back
  };
#endif

  const int pulse_min = 550;
  const int pulse_max = 2550;

  // ## pulse_max offset to adjust servo angle
  // ## tune value if legs are not balance equally.
  int s_offset_max[4][3] = {
      {-30, 365, 200},  // RB
      {-80, -90, 0},   // RF
      {120, 90, 0},    // LF
      {20, -100, -200}  // LB
    };

  // offset values for green sample
  // int s_offset_max[4][3] = {
  //   {0, -50, 0},
  //   {0, 0, 0},
  //   {0, 0, 0},
  //   {0, -100, -50}
  //   };

  const int s_optinv[4][3] = {
      {0, 0, 0},  // ## {dir, dir, dir}
      {1, 0, 0},  // ## {dir, dir, dir}
      {0, 1, 1},  // ## {dir, dir, dir}
      {1, 1, 1}   // ## {dir, dir, dir}
  };

  const int d_constraint_min[3]{-70, 20, 40};   // ## {deg, deg, deg}
  const int d_constraint_max[3]{70, 110, 150};  // ## {deg, deg, deg}

  const int right_back_leg = 0;
  const int right_front_leg = 1;
  const int left_front_leg = 2;
  const int left_back_leg = 3;
  //
  boolean attached;

 public:
  Hardware() : attached(false) {}

  /*
    ::: SETUP :::
  */
  void init_hardware() { attachServos(); }

  boolean attachServos() {
    if (attached) {
      return attached;
    }
    int legs[] = {right_back_leg, left_back_leg, right_front_leg,
                  left_front_leg};
    for (int joint = 0; joint < 3; joint++) {
      for (int i = 0; i < 4; i++) {
        s_output[legs[i]][joint].attach(
            pulse_min, pulse_max + s_offset_max[legs[i]][joint]);
        delay(30);
      }
    }
    return (attached = true);
  }

  void detachServos() {
    if (!attached) return;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        s_output[i][j].detach();
      }
    }
    attached = false;
  }

  void set_leg(int leg, datatypes::Rotator rot) {
    set_joint(leg, 0, rot.yaw);
    set_joint(leg, 1, rot.pitch);
    set_joint(leg, 2, rot.roll);
    // Console.println("set_leg(): yaw="+String(rot.yaw)+",
    // pitch="+String(rot.pitch)+", roll="+String(rot.roll));
  }

  void set_servo(int leg, int joint, float pulse) {
    GPIOservo servo = s_output[leg][joint];
    servo.write(map(pulse, pulse_min, pulse_max, 0, 180));
  }

  void set_max_pw_offset(int index, int joint, int offset) {
    int legs[] = {right_front_leg, right_back_leg, left_front_leg,
                  left_back_leg};

    s_offset_max[legs[index - 1]][joint - 1] = offset;
  }

  int get_max_pw_offset(int index, int joint) {
    int legs[] = {right_front_leg, right_back_leg, left_front_leg,
                  left_back_leg};
    return s_offset_max[legs[index - 1]][joint - 1];
  }

 private:
  void set_joint(int leg, int joint, float deg) {
    int _min = pulse_min;
    int _max = pulse_max + s_offset_max[leg][joint];
    int _inv = s_optinv[leg][joint];
    int _minC = d_constraint_min[joint];
    int _maxC = d_constraint_max[joint];

    if (deg < _minC) deg = _minC;
    // GPIOservo &servo = s_output[leg][joint];
    float pulse = 0;
    if (_inv == 0) {
      pulse = map(deg, _minC, _maxC, _min, _max);
    } else if (_inv == 1) {
      pulse = map(deg, _minC, _maxC, _max, _min);
    }
    // Console.println("set joint() "+String(pulse));
    s_output[leg][joint].writeMicroseconds(pulse);
  }
  /*
    == == == == == == == == == == == == == == ==
    HARDWARE - MPU VARIABLES
    == == == == == == == == == == == == == == ==
    /

    //uint16_t packetSize;    // expected DMP packet size (default 42 bytes)
    //uint8_t fifoBuffer[64]; // FIFO storage buffer

    //Quaternion q;           // [w, x, y, z]         quaternion container
    //float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
    and gravity vector
    //VectorFloat gravity;    // [x, y, z]            gravity vector

    MPU6050 mpu;


    // ::: [Gyroscope/Accelerometer Sensor] FUNCTIONS :::
  */

  void update_mpu_data() {
    /*if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      _sRotation = {ypr[0] * 80 / M_PI,
                    -ypr[1] * 80 / M_PI,
                    ypr[2] * 80 / M_PI
                   };

      if (DEBUG == 0) {
        Console.print(ypr[0] * 60 / M_PI);
        Console.print("/");
        Console.print(-ypr[1] * 60 / M_PI);
        Console.print("/");
        Console.println(ypr[2] * 60 / M_PI);
        }
      }*/
  }

 public:
  void handle_hardware() {
    // update_mpu_data();
  }

  void init_mpu() {
    /*mpu.initialize();
      uint8_t dmp_s = mpu.dmpInitialize();

      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788);

      if (dmp_s == 0) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);

      packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
      if (DEBUG == 1)
        Console.println(":ERROR[" + String(dmp_s) + "]");
      while (1); // ::pause_sketch::
      }*/
  }
#if defined(__DEBUG__)
  void testGPIOservos() {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        s_output[i][j].sweep();
      }
    }
  }
#endif
};
#endif
