#ifndef ACCELEROMETER_MODE_H
#define ACCELEROMETER_MODE_H
#include <Arduino.h>
#include "baseMode.h"
#include "../../Accelerometer/accelerometer.h"

typedef byte accelerometerPrintMode;
#define PRINT_NONE 0
#define PRINT_ACCEL 1
#define PRINT_GYRO 2
#define PRINT_ALL 3

class AccelerometerMode: public Mode {
  private:
    Accelerometer* _accelerometer;

    accelerometerPrintMode currentPrintMode;

    void printReadings();
    void zeroReadings();

  protected:
    controllerMode detectCommand();
  public:
    AccelerometerMode(String* inputString, Accelerometer* MPU9150):Mode(inputString){
      _accelerometer = MPU9150;

      currentPrintMode = PRINT_NONE;
    };
    ~AccelerometerMode() {};

    void printHelp();
    modeResult runMode();
};

#endif