#include "BroydenFunctionMode.h"
BroydenFunctionMode::BroydenFunctionMode(String *inputString,
										 PositionController *positionController,
										 Accelerometer *MPU9150)
	: GradientDecentModeBASE(inputString, positionController, MPU9150, surfaceFitType::poly11)
{
	resetControlValues();
}

controllerMode BroydenFunctionMode::detectCommand()
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

modeResult BroydenFunctionMode::runMode()
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
		if (performBroydenModeControl() == actionState::FINISHED)
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

void BroydenFunctionMode::printHelp()
{
	dualSerial.println("Help in mode: BROYDEN FUNCTION CONTROL MODE. First 2 letters can be accepted");
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

void BroydenFunctionMode::setState(ControlState newState)
{
	GradientDecentModeBASE::setState(newState);

	resetControlValues();
}

void BroydenFunctionMode::resetControlValues()
{
	prevCentreSet = false;
	prevCentrePointIndex = 0;
	centrePointIndex = 0;
	topPointIndex = 1;
	rightPointIndex = 2;
	leftPointIndex = 3;
	bottomPointIndex = 4;

	for (int i = 0; i < 5; i++)
		points[i].valueSet = false;

	controlSpeed = INITAL_CONTROL_SPEED;
	stabilisingEWMAAmplitude = 0;
	moveAmount = RPSToMoveAmount();
	initialNumWavelengths = 0;
	initialNumWavelengthsFound = false;
	currAutoState = AutoState::waiting;
}

// ^^^^^ GENERAL MODE OPERATIONS
// vvvvv CONTROLLER OPERATIONS

actionState BroydenFunctionMode::performBroydenModeControl()
{
	if (!_positionController->bothWeightsZeroed())
		if (_positionController->findZero() != actionState::FINISHED)
			return actionState::WAITING;

	// Set the next collect data points.
	if (currAutoState == AutoState::waiting)
	{
		queueMovePoints();
		currAutoState = AutoState::collectingData;
	}

	if (currAutoState == AutoState::collectingData)
	{
		if (collectData() == actionState::FINISHED)
		{
			currAutoState = AutoState::moving;
		}
	}

	if (currAutoState == AutoState::moving)
	{
		selectBroydenPoint();

		if (moveAmount < MIN_MOVE_AMOUNT)
		{
			_positionController->setDesiredXY(points[centrePointIndex].pos);
			if (_positionController->positionControl() == actionState::FINISHED)
			{
				dualSerial.println("Done becasue of minima");
				return actionState::FINISHED;
			}
		}
		else
		{
			currAutoState = AutoState::waiting;
		}
	}

	return actionState::RUNNING;
}

void BroydenFunctionMode::queueMovePoints()
{
	if (!points[centrePointIndex].valueSet)
	{
		XYPoint currPos = _positionController->getXYPosition();
		pointQueue.enqueue(currPos);
		indexQueue.enqueue(centrePointIndex);
		points[centrePointIndex].setPoint(currPos);
	}

	if (!points[topPointIndex].valueSet)
	{
		XYPoint topPos = points[centrePointIndex].pos;
		topPos.y += moveAmount;
		pointQueue.enqueue(topPos);
		indexQueue.enqueue(topPointIndex);
		points[topPointIndex].setPoint(topPos);
	}

	if (!points[bottomPointIndex].valueSet)
	{
		XYPoint bottomPos = points[centrePointIndex].pos;
		bottomPos.y -= moveAmount;
		pointQueue.enqueue(bottomPos);
		indexQueue.enqueue(bottomPointIndex);
		points[bottomPointIndex].setPoint(bottomPos);
	}

	if (!points[rightPointIndex].valueSet)
	{
		XYPoint rightPos = points[centrePointIndex].pos;
		rightPos.x += moveAmount;
		pointQueue.enqueue(rightPos);
		indexQueue.enqueue(rightPointIndex);
		points[rightPointIndex].setPoint(rightPos);
	}

	if (!points[leftPointIndex].valueSet)
	{
		XYPoint leftPos = points[centrePointIndex].pos;
		leftPos.x -= moveAmount;
		pointQueue.enqueue(leftPos);
		indexQueue.enqueue(leftPointIndex);
		points[leftPointIndex].setPoint(leftPos);
	}
}

void BroydenFunctionMode::storeCollectedData()
{
	XYPoint currPosition = _positionController->getXYPosition();
	dualSerial.print("Got Broyden Reading\t\t\t\t\t\t");
	dualSerial.print(currPosition.x);
	dualSerial.print("\t");
	dualSerial.print(currPosition.y);
	dualSerial.print("\t");
	dualSerial.println(stabilisingEWMAAmplitude);
	points[indexQueue.getHead()].setReading(stabilisingEWMAAmplitude);
	indexQueue.dequeue();
}

void BroydenFunctionMode::selectBroydenPoint()
{
	uint8_t minIndex = -1;
	float minValue = 1000000;
	for (int i = 0; i < 5; i++)
	{
		if (points[i].valueSet && points[i].reading < minValue)
		{
			minIndex = i;
			minValue = points[i].reading;
		}
	}

	// try to avoid unnessicary steps
	if (minIndex != centrePointIndex && points[minIndex].reading >= points[centrePointIndex].reading * 0.97)
	{
		minIndex = centrePointIndex;
	}

	int8_t tempCentrePos = centrePointIndex;
	int8_t tempFoundPos = -1;
	if (minIndex == centrePointIndex)
	{
		dualSerial.println("Selecting centre, reducing size");
		moveAmount /= 2.0;
	}
	else if (minIndex == topPointIndex)
	{
		dualSerial.println("Selecting top, shifting");
		tempFoundPos = bottomPointIndex;
		centrePointIndex = topPointIndex;
		bottomPointIndex = tempCentrePos;
		topPointIndex = tempFoundPos;	 // new Point
		tempFoundPos = centrePointIndex; // for keeping that point when clearing
	}
	else if (minIndex == rightPointIndex)
	{
		dualSerial.println("Selecting right, shifting");
		tempFoundPos = leftPointIndex;
		centrePointIndex = rightPointIndex;
		leftPointIndex = tempCentrePos;
		rightPointIndex = tempFoundPos;	 // new Point
		tempFoundPos = centrePointIndex; // for keeping that point when clearing
	}
	else if (minIndex == bottomPointIndex)
	{
		dualSerial.println("Selecting bottom, shifting");
		tempFoundPos = topPointIndex;
		centrePointIndex = bottomPointIndex;
		topPointIndex = tempCentrePos;
		bottomPointIndex = tempFoundPos; // new Point
		tempFoundPos = centrePointIndex; // for keeping that point when clearing
	}
	else if (minIndex == leftPointIndex)
	{
		dualSerial.println("Selecting left, shifting");
		tempFoundPos = rightPointIndex;
		centrePointIndex = leftPointIndex;
		rightPointIndex = tempCentrePos;
		leftPointIndex = tempFoundPos;	 // new Point
		tempFoundPos = centrePointIndex; // for keeping that point when clearing
	}

	// clear unneeded broyden points
	for (int i = 0; i < 5; i++)
	{
		if (i != tempFoundPos && i != tempCentrePos)
		{
			points[i].clear();
		}
	}
}