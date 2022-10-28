#include "dualSerial.h"

void DualSerial::begin()
{
  _USBSerial->begin(USB_SERIAL_BAUD);
  _BluetoothSerial->begin(BT_SERIAL_BAUD);
}

bool DualSerial::available()
{
  return _USBSerial->available() || _BluetoothSerial->available();
}

void DualSerial::println()
{
  _USBSerial->println();
  _BluetoothSerial->println();
}

void DualSerial::println(const String &str)
{
  _USBSerial->println(str);
  _BluetoothSerial->println(str);
}

void DualSerial::println(double val)
{
  _USBSerial->println(val, NUMBER_OF_DIGITS);
  _BluetoothSerial->println(val, NUMBER_OF_DIGITS);
}

void DualSerial::println(int val)
{
  _USBSerial->println(val);
  _BluetoothSerial->println(val);
}

void DualSerial::println(long val)
{
  _USBSerial->println(val);
  _BluetoothSerial->println(val);
}

void DualSerial::println(unsigned long val)
{
  _USBSerial->println(val);
  _BluetoothSerial->println(val);
}

void DualSerial::print(const String &str)
{
  if (str.length() > 0)
  {
    _USBSerial->print(str);
    _BluetoothSerial->print(str);
  }
}

void DualSerial::print(double val)
{
  _USBSerial->print(val, NUMBER_OF_DIGITS);
  _BluetoothSerial->print(val, NUMBER_OF_DIGITS);
}

void DualSerial::print(int val)
{
  _USBSerial->print(val);
  _BluetoothSerial->print(val);
}

void DualSerial::print(long val)
{
  _USBSerial->print(val);
  _BluetoothSerial->print(val);
}

void DualSerial::print(unsigned long val)
{
  _USBSerial->print(val);
  _BluetoothSerial->print(val);
}

String DualSerial::readString()
{
  UARTClass *currStream;

  if (_USBSerial->available())
    currStream = _USBSerial;
  else
    currStream = _BluetoothSerial;

  String ret = currStream->readString();

  return ret;
}