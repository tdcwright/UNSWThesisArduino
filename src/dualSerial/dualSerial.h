#ifndef DUAL_SERIAL_H
#define DUAL_SERIAL_H
#include <Arduino.h>

#define USB_SERIAL_BAUD 57600
#define BT_SERIAL_BAUD 57600

#define NUMBER_OF_DIGITS 3

class DualSerial
{
public:
  UARTClass *_USBSerial;
  UARTClass *_BluetoothSerial;

  DualSerial(UARTClass *usbSerial, UARTClass *BTSerial) : _USBSerial{usbSerial},
                                                          _BluetoothSerial{BTSerial} {};

  void begin();

  bool available();

  void println();
  void println(const String &str);
  void println(double val);
  void println(int val);
  void println(long val);
  void println(unsigned long val);

  void print(const String &str);
  void print(double val);
  void print(int val);
  void print(long val);
  void print(unsigned long val);

  String readString();
};

#endif