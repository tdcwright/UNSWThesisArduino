#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H
#include <Arduino.h>

class LimitSwitch {
  
  private:
    byte lsPin;
    
  public:
    LimitSwitch(String lsName, byte pin);
    void begin();
    byte getState();
    bool isPressed();
    String switchName;
};

#endif