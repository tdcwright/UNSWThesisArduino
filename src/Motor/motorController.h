#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H
#include <Arduino.h>

#include "../dualSerial/dualSerial.h"
extern DualSerial dualSerial;

typedef byte motorSelect;
#define MOTOR_A 0x1
#define MOTOR_B 0x2
#define MOTOR_BOTH 0x3

typedef int8_t motorSpeed;
#define SPEED_FULL 100
#define SPEED_3QUARTER 75
#define SPEED_HALF 50
#define SPEED_QUARTER 25
#define SPEED_STOP 0
#define SPEED_QUARTER_REVERSE -25
#define SPEED_HALF_REVERSE -50
#define SPEED_3QUARTER_REVERSE -75
#define SPEED_FULL_REVERSE -100

enum class motorDirection
{ // forward = 1, backwards = -1
  forward = 1,
  backwards = -1
};

enum class axisDirection
{ // positive = 1, negative = -1
  positive = 1,
  negative = -1,
  none = 0
};

class MotorController
{

private:
  byte motorAPWMPin;
  byte motorBPWMPin;
  byte forwardPin;
  byte reversePin;

  motorDirection currMotorDirection; // TRUE: Forward, FALSE: Reverse

public:
  MotorController(byte motAPWMPin, byte motBPWMPin, byte forwardDirPin, byte reverseDirPin);
  void begin();
  void stopMotor(motorSelect motor);
  void setSpeed(motorSelect motor, motorSpeed speed);
  void setDirection(axisDirection direction);
  void setDirection(motorDirection direction);
  // Move selected Motor. speed is of range -100->100
  void setMotorVelocity(motorSelect motor, motorSpeed speed);
  void moveMotors(motorSelect motor, motorDirection direction, motorSpeed speed);

  // Set speed of top Motor (only move, no set dir). speed is of range -100->100
  void speedX(motorSpeed speed);

  // Set speed of Bottom Motor (only move, no set dir). speed is of range -100->100
  void speedY(motorSpeed speed);

  // Move top Motor (move and set dir). speed is of range -100->100
  void moveX(motorSpeed speed);

  // Move Bottom Motor (move and set dir). speed is of range -100->100
  void moveY(motorSpeed speed);

  // Move top Motor. speed is of range 0->100
  void moveX(axisDirection direction, motorSpeed speed);

  // Move Bottom Motor. speed is of range 0->100
  void moveY(axisDirection direction, motorSpeed speed);
};

#endif