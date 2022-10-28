#include "bluetooth.h"

Bluetooth::Bluetooth(byte enPin, byte statePin, bool ATMode) {
  _enPin = enPin;
  _statePin = statePin;
  _ATMode = ATMode;
}
void Bluetooth::begin() {
  pinMode(_enPin, OUTPUT);
  pinMode(_statePin, INPUT);

  byte ATModeActivePin = (_ATMode ? HIGH : LOW);
  digitalWrite(_enPin, ATModeActivePin);
}


// TODO: https://forum.arduino.cc/t/hc-05-state-pin/573388/28
bool Bluetooth::connected(){
  return digitalRead(_statePin);
}