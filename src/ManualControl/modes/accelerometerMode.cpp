#include "accelerometerMode.h"

controllerMode AccelerometerMode::detectCommand()
{
	uint inputLength = _inputString->length();
	if (inputLength > 0 && (*_inputString)[inputLength - 1] == '\n')
	{
		char command = (*_inputString)[0];

		switch (command)
		{
		case 'b':
			currentPrintMode = PRINT_ALL;
			break;
		case 'a':
			currentPrintMode = PRINT_ACCEL;
			break;
		case 'g':
			currentPrintMode = PRINT_GYRO;
			break;
		case 's':
			currentPrintMode = PRINT_NONE;
			break;
		case 'r':
			_accelerometer->printHeader();
			break;
		case 'z':
			zeroReadings();
			break;
		case 'e':
			return controllerMode::IDLE;
		case 'h':
		default:
			printHelp();
			break;
		}
	}

	return controllerMode::NO_CHANGE;
}

void AccelerometerMode::zeroReadings()
{
	if (_inputString->length() - 1 == 2)
	{
		char zeroAction = (*_inputString)[1];
		switch (zeroAction)
		{
		case 'b':
			_accelerometer->zeroAll(ACCEL_ZERO_VALS);
			break;
		case 'a':
			_accelerometer->zeroAccel(ACCEL_ZERO_VALS);
			break;
		case 'g':
			_accelerometer->zeroGyro(ACCEL_ZERO_VALS);
			break;
		case 'r':
			_accelerometer->zeroAll(ACCEL_RESET_ZEROS);
			break;
		}
	}
	else
	{
		dualSerial.println("Incorrect string format for zeroing readings");
	}
}

modeResult AccelerometerMode::runMode()
{
	controllerMode newMode = detectCommand();

	if (newMode != controllerMode::NO_CHANGE)
	{
		return modeResult(runResult::MODE_CHANGE, newMode);
	}

	printReadings();

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}

void AccelerometerMode::printReadings()
{
	if (_accelerometer->dataReady())
	{
		_accelerometer->getData();

		switch (currentPrintMode)
		{
		case PRINT_ACCEL:
			_accelerometer->printAccel(true);
			break;
		case PRINT_GYRO:
			_accelerometer->printGyro(true);
			break;
		case PRINT_ALL:
			_accelerometer->printData();
			break;
		}
	}
	else
	{
		dualSerial.println("NO DATA");
	}
}

void AccelerometerMode::printHelp()
{
	dualSerial.println("Help in mode: ACCELEROMETER. Only the first letter command will be accepted");
	dualSerial.println("\th: Help");
	dualSerial.println("\tr: Print header row");
	dualSerial.println("\tb: Print Accelerometer and Gyroscope readings");
	dualSerial.println("\ta: Print Accelerometer readings");
	dualSerial.println("\tg: Print Gyroscope readings");
	dualSerial.println("\ts: Stop printing readings");
	dualSerial.println("\tzb: Zero All");
	dualSerial.println("\tza: Zero accelerometer");
	dualSerial.println("\tzg: Zero gyro");
	dualSerial.println("\tzr: Reset zeros");
	dualSerial.println("\te: return to IDLE");
}