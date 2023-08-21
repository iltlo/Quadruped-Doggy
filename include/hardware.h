#ifndef HARDWARE_H
#define HARDWARE_H

#include "datatypes.h"

void init_hardware();

// void init_servo();

// void init_mpu();

void handle_hardware();

void set_leg(int leg, datatypes::Rotator rot);

void set_joint(int leg, int joint, float deg);

void set_servo(int leg, int joint, float pulse);

// void update_mpu_data();

#endif // HARDWARE_H