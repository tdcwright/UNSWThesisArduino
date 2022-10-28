#include "slidingModeControlMode.h"
SlidingModeControlMode::SlidingModeControlMode(String *inputString,
											   PositionController *positionController,
											   Accelerometer *MPU9150)
	: GradientDecentModeBASE(inputString, positionController, MPU9150, surfaceFitType::poly11)
{
	balancedX = false;
	balancedY = false;

	resetAxisControlValues();
}

controllerMode SlidingModeControlMode::detectCommand()
{
	uint inputLength = _inputString->length();
	if (inputLength > 0 && (*_inputString)[inputLength - 1] == '\n')
	{
		char command1 = (*_inputString)[0];
		char command2 = (*_inputString)[1];

		switch (command1)
		{
		case 'p':
			switch (command2)
			{

			default:
				return GradientDecentModeBASE::detectCommand();
			}
			break;
		default:
			return GradientDecentModeBASE::detectCommand();
		}
	}

	return controllerMode::NO_CHANGE;
}

modeResult SlidingModeControlMode::runMode()
{
	controllerMode newMode = detectCommand();
	if (newMode != controllerMode::NO_CHANGE)
	{
		return modeResult(runResult::MODE_CHANGE, newMode);
	}

	_positionController->directionController();
	collectReadings();
	switch (currState)
	{
	case ControlState::running:
		if (performSlidingModeControl() == actionState::FINISHED)
		{
			setState(ControlState::stopped);
		}
		break;
	default:
		stateMachineOperator();
		break;
	}

	if (printingEachItter)
		printDeviceState();

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}

void SlidingModeControlMode::printHelp()
{
	dualSerial.println("Help in mode: SLIDING MODE CONTROL MODE. First 2 letters can be accepted");
	PositionControlMode::printHelp(false);
	dualSerial.println("\t--------------------------");
	dualSerial.println("\tr: Run controller");
	dualSerial.print("\taa#: Set the Amplitude EWMA Alpha value. Current Value: ");
	dualSerial.println(alphaAmplitudeEWMA);
	dualSerial.print("\tara: Toggle adaptive Readings EWMA Alpha value. Current Value: ");
	dualSerial.println(adaptiveAlphaReadingsEWMA ? "TRUE" : "FALSE");
	dualSerial.print("\tar#: Set the Readings EWMA Alpha value. Current Value: ");
	dualSerial.println(alphaReadingsEWMA);
	dualSerial.println("\tptv: Toggle printing amplitude values");
}

void SlidingModeControlMode::setState(ControlState newState)
{
	GradientDecentModeBASE::setState(newState);
	balancedX = false;
	balancedY = false;
	resetAxisControlValues();
}

// ^^^^^ GENERAL MODE OPERATIONS
// vvvvv CONTROLLER OPERATIONS

actionState SlidingModeControlMode::performSlidingModeControl()
{
	if (!_positionController->bothWeightsZeroed())
		if (_positionController->findZero() != actionState::FINISHED)
			return actionState::WAITING;

	if (!balancedX)
	{
		if (axisMoveSpeed(chooseAxis::x) == actionState::FINISHED)
		{
			dualSerial.println("DONE X");
			balancedX = true;
			resetAxisControlValues();
		}

		return actionState::RUNNING;
	}

	if (!balancedY)
	{
		if (axisMoveSpeed(chooseAxis::y) == actionState::FINISHED)
		{
			dualSerial.println("DONE Y");
			balancedY = true;
			resetAxisControlValues();
		}
		return actionState::RUNNING;
	}

	return actionState::FINISHED;
}

actionState SlidingModeControlMode::axisMoveSpeed(chooseAxis currAxis)
{

	if (moveAmount < 0.5)
	{
		dualSerial.println("Done becasue of minima");
		return actionState::FINISHED;
	}

	if (currControlDirection == 0)
	{
		currControlDirection = getDirectionGuess(currAxis);
		dualSerial.print("Direction guess: ");
		dualSerial.println(currControlDirection);
	}

	if (currAutoState == AutoState::waiting)
	{
		XYPoint currPos = _positionController->getXYPosition();
		if (currAxis == chooseAxis::x)
		{
			currPos.x += currControlDirection * moveAmount;
		}
		else
		{
			currPos.y += currControlDirection * moveAmount;
		}

		_positionController->setDesiredXY(currPos);
		dualSerial.print("Moving to x: ");
		dualSerial.print(currPos.x);
		dualSerial.print(" y: ");
		dualSerial.println(currPos.y);

		currAutoState = AutoState::moving;
	}

	if (currAutoState == AutoState::moving)
	{
		if (_positionController->positionControl() == actionState::FINISHED)
		{
			dualSerial.println("Finished moving");

			currAutoState = AutoState::collectingData;
		}
	}

	if (currAutoState == AutoState::collectingData)
	{
		if (!initialNumWavelengthsFound)
		{
			dualSerial.println("Getting inital counts");
			initialNumWavelengths = numberOfWavelengths;
			initialNumWavelengthsFound = true;
			stabilisingEWMAAmplitude = getEWMAAmplitudeValue();
		}

		stabilisingEWMAAmplitude = getEWMAAmplitudeValue() * STABILISING_EWMA_AMPLITUDE_ALPHA + (1 - STABILISING_EWMA_AMPLITUDE_ALPHA) * stabilisingEWMAAmplitude;

		if (numberOfWavelengths - initialNumWavelengths > NUM_REV_UNTIL_STABLE)
		{

			initialNumWavelengthsFound = false;
			float difference = stabilisingEWMAAmplitude - prevAmplitude;

			if (prevAmplitude != NO_PREVIOUS_READING)
			{

				if (difference < 0)
				{ // Going in the right direction
					dualSerial.println("Right direction");

					wasCorrectDirection = true;
				}
				else
				{
					dualSerial.println("Wrong direction");

					currControlDirection *= -1;

					if (wasCorrectDirection)
					{
						dualSerial.println("Went too far");

						moveAmount /= 2.0;
					}

					wasCorrectDirection = false;
				}
			}
			else
			{
				dualSerial.println("Inital amp found");
			}

			currAutoState = AutoState::waiting;
			prevAmplitude = stabilisingEWMAAmplitude;
		}
	}

	return actionState::RUNNING;
}

void SlidingModeControlMode::resetAxisControlValues()
{
	controlSpeed = INITAL_CONTROL_SPEED;
	ittersOnAxis = 0;
	stabilisingEWMAAmplitude = 0;
	prevAmplitude = NO_PREVIOUS_READING;

	controlDirectionFound = 0;
	currControlDirection = 0;
	moveAmount = RPSToMoveAmount();
	initialNumWavelengths = 0;
	initialNumWavelengthsFound = false;
	wasCorrectDirection = false;
	currAutoState = AutoState::collectingData;
}

short SlidingModeControlMode::getDirectionGuess(chooseAxis currAxis)
{
	short direction = 1;
	double position;
	switch (currAxis)
	{
	case chooseAxis::x:
		position = _positionController->getXPosition();
		break;
	case chooseAxis::y:
		position = _positionController->getYPosition();
		break;
	}

	if (position >= 0)
	{
		direction = -1;
	}

	return direction;
}

double SlidingModeControlMode::RPSToMoveAmount()
{
	// 2rpm: 20mm
	//  4rpm: 10mm
	//  5rpm:  5mm
	double currRotationRate = _MPU9150->getRotationRate();

	return -5 * currRotationRate + 30;
}
