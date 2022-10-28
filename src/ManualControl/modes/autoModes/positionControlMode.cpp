#include "positionControlMode.h"

PositionControlMode::PositionControlMode(String *inputString, PositionController *positionController, Accelerometer *MPU9150, Encoder *encoder) : Mode(inputString)
{
	_MPU9150 = MPU9150;

	_positionController = positionController;

	_encoder = encoder;
	printingEachItter = false;
	printType = 'b';

	encoderDriftIterationsGoal = -1;
	encoderDriftIterationsCompleted = -1;

	setState(ControlState::stopped);
}

controllerMode PositionControlMode::detectCommand()
{
	uint inputLength = _inputString->length();
	if (inputLength > 0 && (*_inputString)[inputLength - 1] == '\n')
	{
		char command1 = (*_inputString)[0];
		char command2 = (*_inputString)[1];

		switch (command1)
		{
		case 'z':
			setState(ControlState::findingZero);
			break;
		case 'd':
			if (inputLength < 3)
				break;

			encoderDriftIterationsGoal = _inputString->substring(1, inputLength - 1).toInt();
			dualSerial.print("Completing ");
			dualSerial.print(encoderDriftIterationsGoal);
			dualSerial.println(" Iterations of Encoder Drift");
			setState(ControlState::findingEncoderDrift);
		case 'k':
			if (inputLength < 4)
				break;
			switch (command2)
			{
			case 'p':
				_positionController->setPControlValue(_inputString->substring(2, inputLength - 1).toDouble());
				dualSerial.print("New *p* control value: ");
				dualSerial.println(_positionController->getPControlValue());
				break;
			case 'i':
				_positionController->setIControlValue(_inputString->substring(2, inputLength - 1).toDouble());
				dualSerial.print("New *i* control value: ");
				dualSerial.println(_positionController->getIControlValue());
				break;
			case 'z':
				_positionController->zeroIntegrals();
				dualSerial.println("Integral values zeroed.");
				break;
			case 's':
				int newMax = _inputString->substring(2, inputLength - 1).toInt();
				_positionController->setMaxSpeed(newMax);
				dualSerial.print("New *max speed* value: ");
				dualSerial.println(newMax);
				break;
			}
			setState(ControlState::positionControl);
			break;
		case 'm':
			if (inputLength < 4)
				break;
			switch (command2)
			{
			case 'x':
				_positionController->setDesiredX(_inputString->substring(2, inputLength - 1).toDouble());
				break;
			case 'y':
				_positionController->setDesiredY(_inputString->substring(2, inputLength - 1).toDouble());
				break;
			}
			setState(ControlState::positionControl);
			break;
		case 'b':
			setMotorAction(command2, true, true);
			break;
		case 'x':
			setMotorAction(command2, true, false);
			break;
		case 'y':
			setMotorAction(command2, false, true);
			break;
		case 'p':
			switch (command2)
			{
			case 't':
				printingEachItter = !printingEachItter;
				switch ((*_inputString)[2])
				{
				case 'a':
					printType = 'a';
					break;
				case 'g':
					printType = 'g';
					break;
				case 'b':
					printType = 'b';
					break;
				}
				if (printingEachItter)
					printHeader();
				break;
			case 'h':
				printHeader();
				break;
			case 'p':
				printMassPositions(true, true);
				break;
			case 'i':
				dualSerial.print("X integral error: ");
				dualSerial.println(_positionController->getXPositionErrIntegral());
				dualSerial.print("Y integral error: ");
				dualSerial.println(_positionController->getYPositionErrIntegral());
				break;
			}
			break;
		case 's':
			_positionController->moveX(SPEED_STOP);
			_positionController->moveY(SPEED_STOP);
			setState(ControlState::stopped);
			break;
		case 'e':
			_positionController->moveX(SPEED_STOP);
			_positionController->moveY(SPEED_STOP);
			return controllerMode::IDLE;
		case 'h':
		default:
			printHelp();
			break;
		}
	}

	return controllerMode::NO_CHANGE;
}

void PositionControlMode::setMotorAction(char command, bool xAxis, bool yAxis)
{
	if (!xAxis && !yAxis)
		return;

	double newPositionX = _positionController->getXPosition();
	double newPositionY = _positionController->getYPosition();

	switch (command)
	{
	case 'p':
		if (xAxis)
			newPositionX = POSITION_CONTROL_POSITIVE_END;
		if (yAxis)
			newPositionY = POSITION_CONTROL_POSITIVE_END;
		break;
	case 'n':
		if (xAxis)
			newPositionX = POSITION_CONTROL_NEGATIVE_END;
		if (yAxis)
			newPositionY = POSITION_CONTROL_NEGATIVE_END;
		break;
	case 'c':
		if (xAxis)
			newPositionX = POSITION_CONTROL_CENTRE;
		if (yAxis)
			newPositionY = POSITION_CONTROL_CENTRE;
		break;
	case 'b':
		if (xAxis)
			newPositionX = X_BALANCED_ZERO;
		if (yAxis)
			newPositionY = Y_BALANCED_ZERO;
		break;
	default:
		dualSerial.println("Invalid control input of Set Motor Action");
		return;
	}

	setState(ControlState::positionControl);

	_positionController->setDesiredX(newPositionX);
	_positionController->setDesiredY(newPositionY);
}

modeResult PositionControlMode::runMode()
{
	controllerMode newMode = detectCommand();
	if (newMode != controllerMode::NO_CHANGE)
	{
		return modeResult(runResult::MODE_CHANGE, newMode);
	}

	_positionController->directionController();
	stateMachineOperator();

	if (printingEachItter)
		printDeviceState();

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}

void PositionControlMode::stateMachineOperator()
{
	switch (currState)
	{
	case ControlState::positionControl:
		if (_positionController->positionControl() == actionState::FINISHED)
		{
			setState(ControlState::stopped);
		}
		break;
	case ControlState::findingZero:
		if (_positionController->findZero() == actionState::FINISHED)
		{
			dualSerial.println("Finished Zeroing Operation. Returning to Centre");
			setMotorAction('c', true, true);
		}
		break;
	case ControlState::findingEncoderDrift:
		if (getEncoderDrift() == actionState::FINISHED)
		{
			dualSerial.println("Finished Encoder Drift Reading. Returning to Centre");
			setMotorAction('c', true, true);
		}
		break;
	case ControlState::stopped:
	default:
		_positionController->moveX(SPEED_STOP);
		_positionController->moveY(SPEED_STOP);
		break;
	}
}

void PositionControlMode::printHelp(bool printHelpHeader)
{
	delay(100);
	if (printHelpHeader)
		dualSerial.println("Help in mode: POSITION CONTROL MODE. First 2 letters can be accepted");
	dualSerial.println("\th: Help");
	dualSerial.println("\tz: Zero both masses");
	dualSerial.println("\td#: Find the encoder drift. # is the number of iterations (0,2,4,6,8)\n"); // NOTE: Zero is just return to the set point

	dualSerial.print("\tmx#: Move x to Position. Current goal: ");
	dualSerial.println(_positionController->getXDesiredPosition());
	dualSerial.print("\tmy#: Move y to Position. Current goal: ");
	dualSerial.println(_positionController->getYDesiredPosition());
	dualSerial.println(" ");
	delay(50);

	dualSerial.println("\t(b/x/y)p: Move both/x/y positive until stop");
	dualSerial.println("\t(b/x/y)n: Move both/x/y negative until stop");
	dualSerial.println("\t(b/x/y)c: Move both/x/y to the centre");
	dualSerial.println("\t(b/x/y)b: Move both/x/y to the balance point");
	delay(50);

	dualSerial.print("\tkp#: Set the proportional constant. Current Value: ");
	dualSerial.println(_positionController->getPControlValue());
	dualSerial.print("\tki#: Set the integral constant. Current Value: ");
	dualSerial.println(_positionController->getIControlValue());
	dualSerial.println("\tkz: Zero integral values\n");
	dualSerial.print("\tks#: Set Max Speed. Current max is: ");
	dualSerial.println(_positionController->getMaxSpeed());
	delay(50);

	dualSerial.println("\tptb: Toggle printing device state. Current state is: ");
	dualSerial.println("\tptg: Toggle printing Gyro. Current state is: ");
	dualSerial.print("\tpta: Toggle printing Accel. Current state is: ");
	dualSerial.println((printingEachItter ? "TRUE" : "FALSE"));
	dualSerial.println("\tph: Output Header once");
	dualSerial.println("\tpp: Output position once");
	dualSerial.println("\tpi: Output Integral Errors once\n");

	dualSerial.println("\ts: Stop movement");
	dualSerial.println("\te: return to IDLE");
	delay(100);
}

void PositionControlMode::printHelp()
{
	printHelp(true);
}

// ^^^^^ GENERAL MODE OPERATIONS
// vvvvv CONTROLLER OPERATIONS

actionState PositionControlMode::getEncoderDrift()
{
	int directionMultiplier = (abs(encoderDriftIterationsCompleted % 2) == 1 ? 1 : -1);

	if (encoderDriftIterationsCompleted < encoderDriftIterationsGoal)
	{
		actionState x = _positionController->moveX(SPEED_FULL * directionMultiplier);
		actionState y = _positionController->moveY(SPEED_FULL * directionMultiplier);

		if (x == actionState::STOPPED && y == actionState::STOPPED)
		{
			encoderDriftIterationsCompleted += 1;
		}

		return actionState::RUNNING;
	}

	dualSerial.print("Encoder drift: x:\t");
	dualSerial.print(_encoder->getXPosition());
	dualSerial.print("mm\t| y:\t");
	dualSerial.println(_encoder->getYPosition());

	encoderDriftIterationsCompleted = -1;
	return actionState::FINISHED;
}

void PositionControlMode::setState(ControlState newState)
{

	currState = newState;
}

ControlState PositionControlMode::getState()
{
	return currState;
}

void PositionControlMode::printHeader()
{
	if (printType == 'b')
		dualSerial.println("t\tax\tay\taz\tgx\tgy\tgz\txPos\typos");
	else if (printType == 'a')
		dualSerial.println("ax\tay\taz");
	else if (printType == 'g')
		dualSerial.println("gx\tgy\tgz");
}

void PositionControlMode::printDeviceState()
{

	if (_MPU9150->dataReady())
	{
		_MPU9150->getData();
		if (printType == 'b')
		{
			dualSerial.print(_MPU9150->collectedTime);
			dualSerial.print("\t");
		}
		if (printType == 'b' || printType == 'a')
			_MPU9150->printAccel();
		if (printType == 'b' || printType == 'g')
			_MPU9150->printGyro();
		if (printType == 'b')
			printMassPositions(false, false);
	}
	dualSerial.println();
}

void PositionControlMode::printMassPositions(bool printHeading, bool newLineAtEnd)
{
	if (printHeading)
		dualSerial.print("XPos:\t");
	dualSerial.print(_positionController->getXPosition());
	dualSerial.print("\t");
	if (printHeading)
		dualSerial.print("YPos:\t");
	dualSerial.print(_positionController->getYPosition());
	if (newLineAtEnd)
		dualSerial.println();
}
