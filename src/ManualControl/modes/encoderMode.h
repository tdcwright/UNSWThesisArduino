#ifndef ENCODER_MODE_H
#define ENCODER_MODE_H
#include <Arduino.h>
#include "baseMode.h"
#include "../../Encoder/encoder.h"

typedef byte encoderPrintMode;
#define PRINT_ENCODER_NONE 0
#define PRINT_ENCODER_1 1
#define PRINT_ENCODER_2 2
#define PRINT_ENCODER_BOTH 3

class EncoderMode: public Mode {
  private:
    Encoder* _encoder;

    encoderPrintMode currentPrintMode;

    void printReadings();
    void zeroReadings();

  protected:
    controllerMode detectCommand();
  public:
    EncoderMode(String* inputString, Encoder* encoders):Mode(inputString){
      _encoder = encoders;

      currentPrintMode = PRINT_ENCODER_NONE;
    };
    ~EncoderMode() {};

    void printHelp();
    modeResult runMode();
};

#endif