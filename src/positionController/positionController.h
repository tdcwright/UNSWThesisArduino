#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H
#include <Arduino.h>

#include "../Encoder/encoder.h"
#include "../LimitSwitch/limitSwitch.h"
#include "../Motor/motorController.h"

#include "../helpers/MovePoints.h"

#include "../dualSerial/dualSerial.h"

#include "defaultPositions.h"

extern DualSerial dualSerial;

#define POSITION_CONTROL_POSITIVE_END 1000
#define POSITION_CONTROL_NEGATIVE_END -1000
#define POSITION_CONTROL_CENTRE 0

#define DISTANCE_TO_CENTRE 100 // mm

#define INTEGRAL_ACTIVATION_DISTANCE 7.5
#define MIN_POSITION_ERROR 0.1
#define MIN_ITTERS_AT_POS 5
#define DEFAULT_PROPORTIONAL_PARAM 12
#define DEFAULT_INTEGRAL_PARAM 10

#define DT_TIMEOUT_LIMIT 500           // Number of milliseconds before the position controller timer times out
#define SINGLE_DIRECTION_MOTOR_HOLD 20 // Number of iteations a direction will hold to allow the other motor to take control

// Defines whether a state is finished or still running. FINISHED, RUNNING, STOPPED or WAITING
enum class actionState
{
  FINISHED,
  RUNNING,
  STOPPED,
  WAITING
};

class PositionController
{

private:
  MotorController *_motorController;

  LimitSwitch *_LSYellow;
  LimitSwitch *_LSPink;
  LimitSwitch *_LSGrey;
  LimitSwitch *_LSWhite;
  Encoder *_encoder;

  uint16_t xZeroCounter; // Keeps track of whether the encoder has been zeroed outside the scope of this function. (Not nessicarily at the limit switch)
  uint16_t yZeroCounter; // Keeps track of whether the encoder has been zeroed outside the scope of this function. (Not nessicarily at the limit switch)

  // Move X mass to non-pulley rod mount and set encoder to zero. RUNNING or FINISHED
  actionState findZeroX();
  // Move Y mass to non-pulley rod mount and set encoder to zero. RUNNING or FINISHED
  actionState findZeroY();

  int directionCount;
  axisDirection currMotorDirection; // A direction state which switches to allow both motors to move in their intended direction
  axisDirection xDesiredDirection;  // A direction state which hold the intended direction of both motors
  axisDirection yDesiredDirection;  // A direction state which hold the intended direction of both motors
  double xDesiredPosition;
  double yDesiredPosition;
  motorSpeed maxSpeed;

  int xIttersAtPos;
  int yIttersAtPos;
  double xPositionErrIntegral;
  double yPositionErrIntegral;
  double pControlValue;
  double iControlValue;
  actionState lastXMoveStateResult;
  actionState lastYMoveStateResult;
  unsigned long posControlTime;
  bool timeIsSet; // Allows integral time to be set on first pos control iteration

  // returns motor speed value. lastMoveState is the result from the previous move state to determine wether to update integral (if motor is waiting, will not update)
  int8_t positionControllerSingleAxis(actionState &lastMoveState, unsigned long &dt, double currPos, double &desiredPos, double &axisIntegral, int &ittersAtPos);

  actionState moveDirection(int8_t speed, bool isXDir, LimitSwitch *posSwitch, LimitSwitch *negSwitch, axisDirection &desiredDirection);

public:
  // Move massess to non-pulley rod mount and set encoder to zero. RUNNING or FINISHED
  actionState findZero();

  // Runs position controller. RUNNING, FINISHED and WAITING (for zero operation to complete)
  actionState positionControl();

  // Manages direction desire of both motors
  void directionController();
  // Move top Motor. speed is of range -100->100. Will not travel in direction if limit switch is active. RUNNING or STOPPED or WAITING (if direction dispute)
  actionState moveX(int8_t speed);

  // Move Bottom Motor. speed is of range -100->100. Will not travel in direction if limit switch is active. RUNNING or STOPPED or WAITING (if direction dispute)
  actionState moveY(int8_t speed);

  PositionController(MotorController *motorController, Encoder *encoder, LimitSwitch *LSYellow, LimitSwitch *LSPink, LimitSwitch *LSGrey, LimitSwitch *LSWhite);

  void setMaxSpeed(int newSpeed);
  int getMaxSpeed();
  void setDesiredXY(XYPoint &newPosition);
  void setDesiredX(double newX);
  void setDesiredY(double newY);
  void setPControlValue(double newP);
  void setIControlValue(double newI);
  double getPControlValue();
  double getIControlValue();
  double getXDesiredPosition();
  double getYDesiredPosition();

  void zeroIntegrals();
  double getXPositionErrIntegral();
  double getYPositionErrIntegral();

  // returns the position of the mass with 0 being at the centre
  double getXPosition();
  // returns the position of the mass with 0 being at the centre
  double getYPosition();
  // returns the position of the mass with 0 being at the centre
  XYPoint getXYPosition();

  bool bothWeightsZeroed(); // Bool for if both weights are currently zeroed (position is known)
};

#endif