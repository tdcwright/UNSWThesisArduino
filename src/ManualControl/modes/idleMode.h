#ifndef IDLE_MODE_H
#define IDLE_MODE_H
#include <Arduino.h>

#include "baseMode.h"

class IdleMode: public Mode {
  protected:
    controllerMode detectCommand();
  public:
    IdleMode(String* inputString):Mode(inputString){};
    ~IdleMode() {};

    void printHelp();
    modeResult runMode();
    };

#endif