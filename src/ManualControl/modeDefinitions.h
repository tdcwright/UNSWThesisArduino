#ifndef MODE_DEFINITIONS_H
#define MODE_DEFINITIONS_H

enum class controllerMode
{
  IDLE = 0,
  MOTOR_CONTROL = 1,
  LIMIT_SWITCH = 2,
  ACCELEROMETER = 3,
  ENCODER = 4,
  POSITION_CONTROL = 5,
  GRADIENT_DECENT_POLY22 = 6,
  GRADIENT_DECENT_POLY21 = 7,
  GRADIENT_DECENT_POLY11 = 8,
  SLIDING_MODE = 9,
  NO_CHANGE = -1
};

#include "../dualSerial/dualSerial.h"
extern DualSerial dualSerial;

#endif