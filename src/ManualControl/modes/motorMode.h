#ifndef MOTOR_MODE_H
#define MOTOR_MODE_H
#include <Arduino.h>
#include "baseMode.h"
#include "../../Motor/motorController.h"

class MotorMode : public Mode
{
private:
  MotorController *_motorController;

  motorSpeed currSpeed;

  void increaseSpeed(uint8_t ammount);
  void decreaseSpeed(uint8_t ammount);
  void runAction(char actionCode);
  char lastAction;

protected:
  controllerMode detectCommand();

public:
  MotorMode(String *inputString, MotorController *motorController) : Mode(inputString)
  {
    _motorController = motorController;
    currSpeed = 50;
    lastAction = 's';
  };
  ~MotorMode(){};

  void printHelp();
  modeResult runMode();
};

#endif