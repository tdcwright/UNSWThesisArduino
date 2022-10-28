#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H
#include <Arduino.h>

#include "../Motor/motorController.h"
#include "../LimitSwitch/limitSwitch.h"
#include "../Accelerometer/accelerometer.h"
#include "../Encoder/encoder.h"
#include "../positionController/positionController.h"

#include "modeDefinitions.h"
#include "modes/baseMode.h"

class ManualController
{

private:
  PositionController *_positionController;
  MotorController *_motorController;

  Accelerometer *_MPU9150;

  LimitSwitch *_LSYellow;
  LimitSwitch *_LSPink;
  LimitSwitch *_LSGrey;
  LimitSwitch *_LSWhite;

  Encoder *_encoder;

  controllerMode currMode;
  Mode *modeController;

  String inputString = "";

  void getInput();
  void setMode(controllerMode newModee, bool deletePrevious = true);

public:
  ManualController(PositionController *positionController, MotorController *motorController, Accelerometer *MPU9150, Encoder *encoder, LimitSwitch *LSYellow, LimitSwitch *LSPink, LimitSwitch *LSGrey, LimitSwitch *LSWhite);

  void begin();
  void runController();
};

#endif