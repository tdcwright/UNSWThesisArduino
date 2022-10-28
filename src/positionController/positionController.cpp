#include "positionController.h"

PositionController::PositionController(MotorController *motorController, Encoder *encoder, LimitSwitch *LSYellow, LimitSwitch *LSPink, LimitSwitch *LSGrey, LimitSwitch *LSWhite)
{
  _motorController = motorController;
  _LSYellow = LSYellow;
  _LSPink = LSPink;
  _LSGrey = LSGrey;
  _LSWhite = LSWhite;
  _encoder = encoder;

  xZeroCounter = UINT16_MAX;
  yZeroCounter = UINT16_MAX;
  directionCount = 0;
  currMotorDirection = axisDirection::none;
  xDesiredDirection = axisDirection::none;
  yDesiredDirection = axisDirection::none;
  maxSpeed = SPEED_FULL;
  xPositionErrIntegral = 0;
  yPositionErrIntegral = 0;
  xIttersAtPos = 0;
  yIttersAtPos = 0;
  pControlValue = DEFAULT_PROPORTIONAL_PARAM;
  iControlValue = DEFAULT_INTEGRAL_PARAM;
  lastXMoveStateResult = actionState::RUNNING;
  lastYMoveStateResult = actionState::RUNNING;
  posControlTime = 0;
}

actionState PositionController::findZero()
{
  actionState x = findZeroX();
  actionState y = findZeroY();

  if (x == actionState::FINISHED && y == actionState::FINISHED)
  {
    xZeroCounter = _encoder->getXZeroCounter();
    yZeroCounter = _encoder->getYZeroCounter();
    return actionState::FINISHED;
  }

  return actionState::RUNNING;
}

actionState PositionController::findZeroX()
{
  actionState moveResult = moveX(maxSpeed);

  if (moveResult == actionState::STOPPED)
  {
    _encoder->zeroEncoder(ENCODER_1);
    return actionState::FINISHED;
  }

  return actionState::RUNNING;
}

actionState PositionController::findZeroY()
{
  actionState moveResult = moveY(maxSpeed);

  if (moveResult == actionState::STOPPED)
  {
    _encoder->zeroEncoder(ENCODER_2);
    return actionState::FINISHED;
  }

  return actionState::RUNNING;
}

actionState PositionController::positionControl()
{
  if (!bothWeightsZeroed())
  {
    if (xZeroCounter == UINT16_MAX)
    {
      xZeroCounter -= 1;
      yZeroCounter -= 1;

      dualSerial.println("Finding Zero then returning to requested position. Please wait.");
    }

    findZero();
    return actionState::WAITING;
  }

  unsigned long now = millis();
  unsigned long dt = now - posControlTime;
  // if (dt > DT_TIMEOUT_LIMIT)
  // {
  //   posControlTime = millis();

  //   xPositionErrIntegral = 0;
  //   yPositionErrIntegral = 0;

  //   lastXMoveStateResult = actionState::RUNNING;
  //   lastYMoveStateResult = actionState::RUNNING;
  //   return actionState::RUNNING;
  // }

  lastXMoveStateResult = moveX(positionControllerSingleAxis(lastXMoveStateResult, dt, getXPosition(), xDesiredPosition, xPositionErrIntegral, xIttersAtPos));
  lastYMoveStateResult = moveY(positionControllerSingleAxis(lastYMoveStateResult, dt, getYPosition(), yDesiredPosition, yPositionErrIntegral, yIttersAtPos));

  posControlTime = now;

  if (lastXMoveStateResult == actionState::STOPPED && lastYMoveStateResult == actionState::STOPPED)
  {
    xPositionErrIntegral = 0;
    yPositionErrIntegral = 0;
    xIttersAtPos = 0;
    yIttersAtPos = 0;
    return actionState::FINISHED;
  }

  return actionState::RUNNING;
}

int8_t PositionController::positionControllerSingleAxis(actionState &lastMoveState, unsigned long &dt, double currPos, double &desiredPos, double &axisIntegral, int &ittersAtPos)
{
  double posErr = desiredPos - currPos;
  // dualSerial.print("Pos Control Error: ");
  // dualSerial.println(posErr);

  if (abs(posErr) < MIN_POSITION_ERROR)
  {
    if (ittersAtPos > MIN_ITTERS_AT_POS)
    {
      return SPEED_STOP;
    }
    ittersAtPos++;
  }
  else
  {
    ittersAtPos = 0;
  }

  if (abs(posErr) < INTEGRAL_ACTIVATION_DISTANCE)
  {
    if (lastMoveState != actionState::WAITING)
      axisIntegral += posErr * (dt / 1000.0);
  }
  else
  {
    axisIntegral = 0;
  }

  double controlEffort = posErr * pControlValue + axisIntegral * iControlValue;

  if (abs(controlEffort) > maxSpeed)
    controlEffort = abs(controlEffort) / (controlEffort * 1.0) * maxSpeed; // Don't allow speeds over +/- maxSpeed

  return (int8_t)controlEffort;
}

void PositionController::directionController()
{
  if (xDesiredDirection == axisDirection::none)
  {
    currMotorDirection = yDesiredDirection;
    _motorController->setDirection(currMotorDirection);
    directionCount = 0;
    return;
  }

  if (yDesiredDirection == axisDirection::none)
  {
    currMotorDirection = xDesiredDirection;
    _motorController->setDirection(currMotorDirection);
    directionCount = 0;
    return;
  }

  if (xDesiredDirection == yDesiredDirection)
  {
    currMotorDirection = xDesiredDirection;
    _motorController->setDirection(currMotorDirection);
    directionCount = 0;
    return;
  }

  if (directionCount >= SINGLE_DIRECTION_MOTOR_HOLD)
  {
    currMotorDirection = (currMotorDirection == axisDirection::positive ? axisDirection::negative : axisDirection::positive);
    _motorController->setDirection(currMotorDirection);
    directionCount = 0;
  }
  else
  {
    directionCount += 1;
  }
}

actionState PositionController::moveX(int8_t speed)
{
  return moveDirection(speed, true, _LSPink, _LSYellow, xDesiredDirection);
}

actionState PositionController::moveY(int8_t speed)
{
  return moveDirection(speed, false, _LSWhite, _LSGrey, yDesiredDirection);
}

actionState PositionController::moveDirection(int8_t speed, bool isXDir, LimitSwitch *posSwitch, LimitSwitch *negSwitch, axisDirection &desiredDirection)
{

  if (abs(speed) > maxSpeed)
    speed = abs(speed) / (speed * 1.0) * maxSpeed; // Don't allow speeds over +/- maxSpeed

  if (speed == SPEED_STOP)
  {
    if (isXDir)
      _motorController->speedX(SPEED_STOP);
    else
      _motorController->speedY(SPEED_STOP);
    desiredDirection = axisDirection::none;
    return actionState::STOPPED;
  }

  desiredDirection = (speed > 0 ? axisDirection::positive : axisDirection::negative);

  if ((posSwitch->isPressed() && desiredDirection == axisDirection::positive) || (negSwitch->isPressed() && desiredDirection == axisDirection::negative))
  {
    if (isXDir)
      _motorController->speedX(SPEED_STOP);
    else
      _motorController->speedY(SPEED_STOP);

    desiredDirection = axisDirection::none;
    return actionState::STOPPED;
  }

  if (desiredDirection != currMotorDirection)
  {
    if (isXDir)
      _motorController->speedX(SPEED_STOP);
    else
      _motorController->speedY(SPEED_STOP);
    return actionState::WAITING;
  }

  if (isXDir)
    _motorController->speedX(speed);
  else
    _motorController->speedY(speed);
  return actionState::RUNNING;
}

double PositionController::getXPosition()
{
  return _encoder->getXPosition() + DISTANCE_TO_CENTRE;
}

double PositionController::getYPosition()
{
  return _encoder->getYPosition() + DISTANCE_TO_CENTRE;
}

XYPoint PositionController::getXYPosition()
{
  return XYPoint(getXPosition(), getYPosition());
}

bool PositionController::bothWeightsZeroed()
{
  return xZeroCounter == _encoder->getXZeroCounter() && yZeroCounter == _encoder->getYZeroCounter();
}

void PositionController::setMaxSpeed(int newSpeed)
{
  if (abs(newSpeed) > SPEED_FULL)
    newSpeed = abs(newSpeed) * SPEED_FULL; // Don't allow speeds over +/- 100%
  maxSpeed = newSpeed;
}

int PositionController::getMaxSpeed()
{
  return maxSpeed;
}

void PositionController::zeroIntegrals()
{
  xPositionErrIntegral = 0;
  yPositionErrIntegral = 0;
}

void PositionController::setDesiredXY(XYPoint &newPosition)
{
  setDesiredX(newPosition.x);
  setDesiredY(newPosition.y);
}

void PositionController::setDesiredX(double newX)
{
  xDesiredPosition = newX;
}

void PositionController::setDesiredY(double newY)
{
  yDesiredPosition = newY;
}

void PositionController::setPControlValue(double newP)
{
  pControlValue = newP;
}

void PositionController::setIControlValue(double newI)
{
  iControlValue = newI;
}

double PositionController::getPControlValue()
{
  return pControlValue;
}

double PositionController::getIControlValue()
{
  return iControlValue;
}

double PositionController::getXPositionErrIntegral()
{
  return xPositionErrIntegral;
}

double PositionController::getYPositionErrIntegral()
{
  return yPositionErrIntegral;
}

double PositionController::getXDesiredPosition()
{
  return xDesiredPosition;
}

double PositionController::getYDesiredPosition()
{
  return yDesiredPosition;
}