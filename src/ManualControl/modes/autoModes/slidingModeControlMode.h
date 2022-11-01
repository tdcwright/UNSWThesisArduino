#ifndef SLIDING_MODE_CONTROL_MODE_H
#define SLIDING_MODE_CONTROL_MODE_H
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

enum class chooseAxis
{
	x,
	y
};

class SlidingModeControlMode : public GradientDecentModeBASE
{
private:
	void resetAxisControlValues();

protected:
	bool balancedX;
	bool balancedY;
	int ittersOnAxis;
	bool wasCorrectDirection;

	float prevAmplitude;
	double moveAmount;

	int controlSpeed;
	int controlDirectionFound;
	int currControlDirection;

	// RUNNING, FINISHED or WAITING (if zeroing)
	actionState performSlidingModeControl();

	// Estimates the best speed to go in a direction to minimise error. RUNNING, FINISHED
	actionState axisMoveSpeed(chooseAxis currAxis);
	// Based on where the massess currently is, determine a direction
	short getDirectionGuess(chooseAxis currAxis);

	virtual void setState(ControlState newState);
	controllerMode detectCommand();

	void queueMovePoints(){};

public:
	SlidingModeControlMode(String *inputString, PositionController *positionController, Accelerometer *MPU9150);
	~SlidingModeControlMode(){};

	void printHelp();
	modeResult runMode();
};

#endif