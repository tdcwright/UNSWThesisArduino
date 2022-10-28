#ifndef LIMIT_SWITCH_MODE_H
#define LIMIT_SWITCH_MODE_H
#include <Arduino.h>
#include "baseMode.h"
#include "../../LimitSwitch/limitSwitch.h"

class LimitSwitchMode: public Mode {
  private:
    LimitSwitch* _LSYellow;
    LimitSwitch* _LSPink;
    LimitSwitch* _LSGrey;
    LimitSwitch* _LSWhite;

    bool limitSwitchChosen;
    LimitSwitch* activeLimitSwitch;

    bool printStateToggle;

  protected:
    controllerMode detectCommand();
  public:
    LimitSwitchMode(String* inputString, LimitSwitch* LSYellow, LimitSwitch* LSPink, LimitSwitch* LSGrey, LimitSwitch* LSWhite):Mode(inputString){
      _LSYellow = LSYellow;
      _LSPink = LSPink;
      _LSGrey = LSGrey;
      _LSWhite = LSWhite;

      limitSwitchChosen = false;
      printStateToggle = false;
    };

    ~LimitSwitchMode() {};

    void printHelp();
    modeResult runMode();
};

#endif