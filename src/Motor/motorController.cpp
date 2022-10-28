#include "motorController.h"

MotorController::MotorController(byte motAPWMPin, byte motBPWMPin, byte forwardDirPin, byte reverseDirPin)
{
  motorAPWMPin = motAPWMPin;
  motorBPWMPin = motBPWMPin;
  forwardPin = forwardDirPin;
  reversePin = reverseDirPin;

  currMotorDirection = motorDirection::forward;
}
void MotorController::begin()
{
  pinMode(motorAPWMPin, OUTPUT);
  pinMode(motorBPWMPin, OUTPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);

  stopMotor(MOTOR_BOTH);
  setDirection(currMotorDirection);
}

void MotorController::setSpeed(motorSelect motor, motorSpeed speed)
{
  if (speed < 0)
  {
    dualSerial.print("*ERROR* Recieved negative speed: ");
    dualSerial.println(speed);
    return;
  }

  uint32_t pwmValue = map(speed, -1, 101, 0, 255);

  if ((motor & MOTOR_A) > 0)
  {
    analogWrite(motorAPWMPin, pwmValue);
  }
  if ((motor & MOTOR_B) > 0)
  {
    analogWrite(motorBPWMPin, pwmValue);
  }
}

void MotorController::stopMotor(motorSelect motor)
{
  setSpeed(motor, SPEED_STOP);
}

void MotorController::setDirection(axisDirection direction)
{
  switch (direction)
  {
  case axisDirection::negative:
    setDirection(motorDirection::backwards);
    break;

  case axisDirection::positive:
  case axisDirection::none:
  default:
    setDirection(motorDirection::forward);
    break;
  }
}

void MotorController::setDirection(motorDirection direction)
{
  byte directionAValue = direction == motorDirection::forward ? HIGH : LOW;
  byte directionBValue = direction == motorDirection::forward ? LOW : HIGH;

  currMotorDirection = direction;

  digitalWrite(forwardPin, directionAValue);
  digitalWrite(reversePin, directionBValue);
}

void MotorController::moveMotors(motorSelect motor, motorDirection direction, motorSpeed speed)
{
  setDirection(direction);
  setSpeed(motor, speed);
}

void MotorController::speedX(motorSpeed speed)
{
  setSpeed(MOTOR_A, abs(speed));
}

void MotorController::speedY(motorSpeed speed)
{
  setSpeed(MOTOR_B, abs(speed));
}

void MotorController::moveX(motorSpeed speed)
{
  setMotorVelocity(MOTOR_A, speed);
}

void MotorController::moveY(motorSpeed speed)
{
  setMotorVelocity(MOTOR_B, speed);
}

void MotorController::moveX(axisDirection direction, motorSpeed speed)
{
  // setSpeed(MOTOR_B, SPEED_STOP);
  setDirection((motorDirection)direction);
  setSpeed(MOTOR_A, speed);
}

void MotorController::moveY(axisDirection direction, motorSpeed speed)
{
  // setSpeed(MOTOR_A, SPEED_STOP);
  setDirection((motorDirection)direction);
  setSpeed(MOTOR_B, speed);
}

void MotorController::setMotorVelocity(motorSelect motor, motorSpeed speed)
{
  setDirection((speed >= 0 ? motorDirection::forward : motorDirection::backwards));

  setSpeed(motor, abs(speed));
}
