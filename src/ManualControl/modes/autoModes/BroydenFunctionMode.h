#ifndef BROYDEN_FUNCTION_MODE_H
#define BROYDEN_FUNCTION_MODE_H
#include <Arduino.h>

#include "../../../Accelerometer/accelerometer.h"
#include "../../../Encoder/encoder.h"
#include "../../../positionController/positionController.h"

#include "../../../dualSerial/dualSerial.h"

#include "../baseMode.h"
#include "gradientDecentMode.h"

#include "../../../helpers/circularArray.h"
#include "../../../helpers/MovePoints.h"

extern DualSerial dualSerial;

#define INITAL_CONTROL_SPEED SPEED_3QUARTER
#define CONTROL_SPEED_MIN_THRESHOLD 45

#define NO_PREVIOUS_READING -1111111

#define MIN_MOVE_AMOUNT 0.5

struct BroydenPoint
{
	bool valueSet;
	XYPoint pos;
	float reading;

	BroydenPoint() : pos(0, 0)
	{
		valueSet = false;
		reading = 0;
	}

	void set(XYPoint p, float value)
	{
		pos = p;
		reading = value;
		valueSet = true;
	}

	void setPoint(XYPoint p)
	{
		pos = p;
	}

	void setReading(float value)
	{
		reading = value;
		valueSet = true;
	}

	void clear()
	{
		reading = 0;
		valueSet = false;
	}
};

class BroydenFunctionMode : public GradientDecentModeBASE
{
protected:
	BroydenPoint points[5];
	bool prevCentreSet;
	uint8_t prevCentrePointIndex;
	uint8_t centrePointIndex;
	uint8_t topPointIndex;
	uint8_t rightPointIndex;
	uint8_t leftPointIndex;
	uint8_t bottomPointIndex;
	ArduinoQueue<uint8_t> indexQueue;

	double moveAmount;

	int controlSpeed;

	// RUNNING, FINISHED or WAITING (if zeroing)
	actionState performBroydenModeControl();

	virtual void setState(ControlState newState);
	controllerMode detectCommand();

	void queueMovePoints();

	void resetControlValues();

	void selectBroydenPoint();

	virtual void storeCollectedData();

public:
	BroydenFunctionMode(String *inputString, PositionController *positionController, Accelerometer *MPU9150);
	~BroydenFunctionMode(){};

	void printHelp();
	modeResult runMode();
};

#endif