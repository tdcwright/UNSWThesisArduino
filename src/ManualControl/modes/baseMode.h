#ifndef BASE_MODE_H
#define BASE_MODE_H
#include <Arduino.h>

#include "../modeDefinitions.h"

enum class ControlState
{
	stopped,
	findingZero,
	positionControl,
	findingEncoderDrift,
	running
};

enum class runResult
{
	MODE_SUCESS,
	MODE_CHANGE
};

typedef struct modeResult
{
	runResult currResult;
	controllerMode newMode;

	modeResult(runResult res, controllerMode mode) : currResult(res), newMode(mode){};
} modeResult;

class Mode
{
protected:
	String *_inputString;
	virtual controllerMode detectCommand() = 0;

public:
	Mode(String *inputString) : _inputString(inputString){};
	virtual ~Mode(){};

	virtual void printHelp() = 0;
	virtual modeResult runMode() = 0;
};

#endif