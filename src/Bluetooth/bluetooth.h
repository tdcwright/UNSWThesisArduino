#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#include <Arduino.h>

// ON COM PORT 4

class Bluetooth
{

private:
  byte _enPin;
  byte _statePin;
  bool _ATMode;

public:
  Bluetooth(byte enPin, byte statePin, bool ATMode = false);
  void begin();
  bool connected();
};

#endif