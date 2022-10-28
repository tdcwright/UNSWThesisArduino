#ifndef POSITION_CONTROLLER_MODE_H
#define POSITION_CONTROLLER_MODE_H
#include <Arduino.h>

#include "../../../Accelerometer/accelerometer.h"
#include "../../../Encoder/encoder.h"
#include "../../../positionController/positionController.h"

#include "../../../dualSerial/dualSerial.h"

#include "../baseMode.h"

extern DualSerial dualSerial;

class PositionControlMode : public Mode
{
private:
	Encoder *_encoder;

	// Number of iterations defines the number of times a mass will move to the edge. 0 means move from current location to zero and stop. RUNNING or FINISHED
	actionState getEncoderDrift();
	int encoderDriftIterationsGoal;		 // Number of iterations the drift should be taken off
	int encoderDriftIterationsCompleted; // Current number of iterations taken

protected:
	Accelerometer *_MPU9150;
	PositionController *_positionController;

	ControlState currState;

	bool printingEachItter; // Bool for if the device state get printed each itter
	char printType;			// What data to print each itter

	controllerMode detectCommand();

	// based on move command, performs forward 'p', reverse 'n' or center 'c' command for each of the motors.
	void setMotorAction(char command, bool xAxis, bool yAxis);

	void printHelp(bool printHelpHeader);

	void stateMachineOperator();

public:
	PositionControlMode(String *inputString, PositionController *positionController, Accelerometer *MPU9150, Encoder *encoder);
	~PositionControlMode(){};

	// Sets the state to newState, resets nessciary variables (eg, time counter for pos control)
	virtual void setState(ControlState newState);
	ControlState getState();

	virtual void printHeader();
	// Prints all device data (ACCEL, GYRO, Zero State, XPOS, YPOS)
	void printDeviceState();
	// Print just the Zero State,  XPOS and YPOS
	void printMassPositions(bool printHeading, bool newLineAtEnd);

	void printHelp();
	modeResult runMode();
};

#endif