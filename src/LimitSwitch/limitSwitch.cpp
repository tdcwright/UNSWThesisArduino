#include "limitSwitch.h"

LimitSwitch::LimitSwitch(String lsName, byte pin) {
  lsPin = pin;
  switchName = lsName;
}
void LimitSwitch::begin() {
  pinMode(lsPin, INPUT_PULLUP);
}

byte LimitSwitch::getState() {
  return digitalRead(lsPin);
}

bool LimitSwitch::isPressed() {
  return (getState() == HIGH);
}