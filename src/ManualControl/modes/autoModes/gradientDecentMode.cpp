#include "gradientDecentMode.h"

template <int numVars, int numReadings>
GradientDecentModeBASE<numVars, numReadings>::GradientDecentModeBASE(String *inputString,
																	 PositionController *positionController,
																	 Accelerometer *MPU9150,
																	 surfaceFitType sFitType)
	: PositionControlMode(inputString, positionController, MPU9150, NULL),
	  surface(sFitType),
	  pastGY(NUMBER_OF_PAST_GYRO_VALUES), // Initalise two circular arrays
	  initialPosition(0, 0)
#if USE_X_AND_Y_READINGS_FOR_ERROR
	  ,
	  pastGX(NUMBER_OF_PAST_GYRO_VALUES) // Initalise two circular arrays
#endif
{
	fitType = sFitType;
	currAutoState = AutoState::waiting;
	dataCollectionIteration = 0;
	returningToCentre = false;
	nextPostionFoundAndSet = false;

	printedSurfaceState = false;

	adaptiveAlphaReadingsEWMA = true;
	alphaReadingsEWMA = DEFAULT_SLOW_READINGS_EWMA_ALPHA;
	runningAveragesActivated = false;

	pastTime = 0;
	wavelengthTime = 0;
	numberOfWavelengths = 0;

	alphaAmplitudeEWMA = DEFAULT_AMPLITUDE_EWMA_ALPHA;
	gyRunningAverage = 0;
	yAmplitudeAveragesActivated = false;
	gyAmplitudeRateEWMA = 0;
	gyLastAmplitudeEWMA = 0;
	gyAmplitudeEWMA = 0;
#if USE_X_AND_Y_READINGS_FOR_ERROR
	gxRunningAverage = 0;
	xAmplitudeAveragesActivated = false;
	gxAmplitudeRateEWMA = 0;
	gxLastAmplitudeEWMA = 0;
	gxAmplitudeEWMA = 0;
#endif
}

template <int numVars, int numReadings>
controllerMode GradientDecentModeBASE<numVars, numReadings>::detectCommand()
{
	uint inputLength = _inputString->length();
	if (inputLength > 0 && (*_inputString)[inputLength - 1] == '\n')
	{
		char command1 = (*_inputString)[0];
		char command2 = (*_inputString)[1];

		switch (command1)
		{
		case 'r':
			setState(ControlState::running);
			break;
		case 'd': // Override encoder drift measurement input
			break;
		case 'a':
			switch (command2)
			{
			case 'r':
				switch ((*_inputString)[2])
				{
				case 'a':
					adaptiveAlphaReadingsEWMA = !adaptiveAlphaReadingsEWMA;
					break;
				default:
					alphaReadingsEWMA = _inputString->substring(2, inputLength - 1).toDouble();
					dualSerial.print("New *Readings EWPM alpha* value: ");
					dualSerial.println(alphaReadingsEWMA);
					break;
				}
				break;
			case 'a':
				alphaReadingsEWMA = _inputString->substring(2, inputLength - 1).toDouble();
				dualSerial.print("New *Amplitude EWPM alpha* value: ");
				dualSerial.println(alphaAmplitudeEWMA);
				break;

			default:
				break;
			}
			break;
		case 'p':
			switch (command2)
			{
			case 't':
				switch ((*_inputString)[2])
				{
				case 'v':
					printingEachItter = !printingEachItter;
					printType = 'v';
					if (printingEachItter)
						printHeader();
					break;
				default:
					return PositionControlMode::detectCommand();
				}
				break;
			case 's':
				printSurface();
				break;
			default:
				return PositionControlMode::detectCommand();
			}
			break;
		default:
			return PositionControlMode::detectCommand();
		}
	}

	return controllerMode::NO_CHANGE;
}

template <int numVars, int numReadings>
modeResult GradientDecentModeBASE<numVars, numReadings>::runMode()
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
		if (performGradDecent() == actionState::FINISHED)
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

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::printHelp()
{
	dualSerial.println("Help in mode: GRADIENT DECENT MODE. First 2 letters can be accepted");
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
	dualSerial.println("\tps: Print current surface coefficients");
}

// ^^^^^ GENERAL MODE OPERATIONS
// vvvvv CONTROLLER OPERATIONS

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::collectReadings()
{
	if (!_MPU9150->dataReady())
		return;

	_MPU9150->getData();

	if (!runningAveragesActivated)
	{
		gyRunningAverage = _MPU9150->gy;
#if USE_X_AND_Y_READINGS_FOR_ERROR
		gxRunningAverage = _MPU9150->gx;
#endif

		wavelengthTime = 0;
		runningAveragesActivated = true;
	}

	// Check if new wavelength has passed
	bool newWavelength = false;
	if (!USE_CROSSED_AVERAGE)
	{
		if (newWavelength)
		{
			pastTime = _MPU9150->collectedTime;
		}
		else
		{
			wavelengthTime += _MPU9150->collectedTime - pastTime;
			pastTime = _MPU9150->collectedTime;

			double currRotationRate = _MPU9150->getRotationRate();
			if (wavelengthTime >= 500.0 / abs(currRotationRate))
			{
				newWavelength = true;
				wavelengthTime = 0;
				numberOfWavelengths++;
			}
		}
	}

	collectReadingsSingleAxis(_MPU9150->gy, pastGY, gyRunningAverage, yAmplitudeAveragesActivated, gyAmplitudeEWMA, gyLastAmplitudeEWMA, gyAmplitudeRateEWMA, newWavelength);
#if USE_X_AND_Y_READINGS_FOR_ERROR
	collectReadingsSingleAxis(_MPU9150->gx, pastGX, gxRunningAverage, xAmplitudeAveragesActivated, gxAmplitudeEWMA, gxLastAmplitudeEWMA, gxAmplitudeRateEWMA, newWavelength);
#endif
}
template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::collectReadingsSingleAxis(float newReading,
																			 CircularArray<float> &pastReadings,
																			 float &runningAverage,
																			 bool &amplitudeAvgSet,
																			 float &amplitudeEWMA,
																			 float &lastAmplitudeEWMA,
																			 float &amplitudeRateEWMA,
																			 bool &newWavelength)
{
	runningAverage = newReading * getAlphaReadingsEWMA() + (1 - getAlphaReadingsEWMA()) * runningAverage;
	pastReadings.addValue(newReading);

	if (newWavelength || (USE_CROSSED_AVERAGE && checkbackCrossedAverage(pastReadings, runningAverage)))
	{
		minMaxVals<float> currMinMax = pastReadings.getMaxMin();
		float currAmplitude = currMinMax.maxValue - currMinMax.minValue;

		if (!amplitudeAvgSet)
		{
			amplitudeEWMA = currAmplitude;
			amplitudeAvgSet = true;
		}

		lastAmplitudeEWMA = amplitudeEWMA;
		amplitudeEWMA = currAmplitude * alphaAmplitudeEWMA + (1 - alphaAmplitudeEWMA) * amplitudeEWMA;

		float currAmplitudeRate = amplitudeEWMA - lastAmplitudeEWMA;
		amplitudeRateEWMA = currAmplitudeRate * DEFAULT_AMPLITUDE_RATE_EWMA_ALPHA + (1 - DEFAULT_AMPLITUDE_RATE_EWMA_ALPHA) * amplitudeRateEWMA;
		pastReadings.reset();
	}
}

template <int numVars, int numReadings>
bool GradientDecentModeBASE<numVars, numReadings>::checkbackCrossedAverage(CircularArray<float> &pastReadings, float &averageReadings)
{
	if (pastReadings.numElements() < NUM_OF_CHECKBACK_VALUES)
		return false;

	if (pastReadings.getFILO(NUM_OF_CHECKBACK_VALUES) < averageReadings && pastReadings.getFILO(0) > averageReadings)
		return true;

	return false;
}

template <int numVars, int numReadings>
actionState GradientDecentModeBASE<numVars, numReadings>::performGradDecent()
{
	// Set the next collect data points.
	if (currAutoState == AutoState::waiting)
	{
		initialPosition = _positionController->getXYPosition();
		queueMovePoints();
		surface.reset();
		currAutoState = AutoState::collectingData;
	}

	if (currAutoState == AutoState::collectingData)
	{
		if (collectData() == actionState::FINISHED)
		{

			currAutoState = AutoState::movingToMinima;
		}
	}

	if (currAutoState == AutoState::movingToMinima)
	{

		if (!surface.operationCompleted)
		{
			surface.performLeastSquaresOperation();
			if (!printedSurfaceState && (!surface.validReductionOperation || !surface.validSurface))
			{
				if (!surface.validReductionOperation)
					dualSerial.println("*ERROR* Invalid Row Echelon Operation");
				if (!surface.validSurface)
				{
					dualSerial.println("*ERROR* Invalid Surface");
					surface.printSurfaceCoefficients();
				}
				printedSurfaceState = true;
			}

			dualSerial.println("Competed surface operation");
		}

		if (surface.validReductionOperation &&
			surface.validSurface)
		{
			if (!printedSurfaceState)
			{
				surface.printSurfaceCoefficients();
				printedSurfaceState = true;
			}

			if (goToNextPosition() == actionState::FINISHED)
				currAutoState = AutoState::finishedMoving;
		}
		else
		{
			if (!returningToCentre)
			{
				dualSerial.println("Invalid surface, moving to the centre and trying again");
				returningToCentre = true;
			}

			_positionController->setDesiredX(0);
			_positionController->setDesiredY(0);

			if (_positionController->positionControl() == actionState::FINISHED)
			{
				returningToCentre = false;
				dataCollectionIteration--; // undo last iteration
				currAutoState = AutoState::waiting;
				printedSurfaceState = false;
			}
		}
	}

	// In the case that have moved to end, and still vibrating
	if (dataCollectionIteration > 0 && currAutoState == AutoState::finishedMoving)
	{
		// Finish even if not at minima
		printingEachItter = false;
		return actionState::FINISHED;

		if (getEWMAAmplitudeValue() > STABLE_AMPLITUDE_THRESHOLD)
		{
			currAutoState = AutoState::waiting;
		}
		else
		{
			return actionState::FINISHED;
		}
	}

	return actionState::RUNNING;
}

template <int numVars, int numReadings>
actionState GradientDecentModeBASE<numVars, numReadings>::collectData()
{
	if (pointQueue.isEmpty())
	{
		dataCollectionIteration++;
		return actionState::FINISHED;
	}

	XYPoint movePoint = pointQueue.getHead();
	_positionController->setDesiredXY(movePoint);
	if (_positionController->positionControl() == actionState::FINISHED)
	{
		if (!isAmplitudeStable())
		{
			return actionState::WAITING;
		}

		storeCollectedData();
		pointQueue.dequeue();
	}

	return actionState::RUNNING;
}

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::storeCollectedData()
{
	XYPoint currPosition = _positionController->getXYPosition();
	dualSerial.print("adding readings to surface\t\t\t\t\t\t");
	dualSerial.print(currPosition.x);
	dualSerial.print("\t");
	dualSerial.print(currPosition.y);
	dualSerial.print("\t");
	dualSerial.println(stabilisingEWMAAmplitude);
	surface.addReadings(currPosition.x, currPosition.y, stabilisingEWMAAmplitude, AMPLITUDE_READINGS_NORMALISATION_POWER);
}

template <int numVars, int numReadings>
actionState GradientDecentModeBASE<numVars, numReadings>::goToNextPosition()
{
	if (!nextPostionFoundAndSet)
	{
		XYPoint directionOfDecent = surface.directionOfSteepestDecent(initialPosition);
		_positionController->setDesiredXY(directionOfDecent);
		nextPostionFoundAndSet = true;
	}

	if (_positionController->positionControl() == actionState::FINISHED)
	{
		nextPostionFoundAndSet = false;
		return actionState::FINISHED;
	}

	return actionState::RUNNING;
}

template <int numVars, int numReadings>
bool GradientDecentModeBASE<numVars, numReadings>::isAmplitudeStable()
{
	if (!initialNumWavelengthsFound)
	{
		initialNumWavelengths = numberOfWavelengths;
		initialNumWavelengthsFound = true;
		stabilisingEWMAAmplitude = getEWMAAmplitudeValue();
	}

	stabilisingEWMAAmplitude = getEWMAAmplitudeValue() * STABILISING_EWMA_AMPLITUDE_ALPHA + (1 - STABILISING_EWMA_AMPLITUDE_ALPHA) * stabilisingEWMAAmplitude;

	if (isnan(stabilisingEWMAAmplitude) ||
		isinf(stabilisingEWMAAmplitude) ||
		stabilisingEWMAAmplitude > 4294967040.0 ||
		stabilisingEWMAAmplitude < -4294967040.0) // overflow
	{
		initialNumWavelengthsFound = false;
		return false;
	}

	if (numberOfWavelengths - initialNumWavelengths > NUM_REV_UNTIL_STABLE)
	{
		initialNumWavelengthsFound = false;
		return true;
	}

	return false;
}

template <int numVars, int numReadings>
float GradientDecentModeBASE<numVars, numReadings>::getEWMAAmplitudeValue()
{
#if USE_X_AND_Y_READINGS_FOR_ERROR
	return (gxAmplitudeEWMA + gyAmplitudeEWMA) / 2;
#else
	return gyAmplitudeEWMA;
#endif
}

template <int numVars, int numReadings>
float GradientDecentModeBASE<numVars, numReadings>::getEWMAAmplitudeRateValue()
{
#if USE_X_AND_Y_READINGS_FOR_ERROR
	return (gxAmplitudeRateEWMA + gyAmplitudeRateEWMA) / 2;
#else
	return gyAmplitudeRateEWMA;
#endif
}

template <int numVars, int numReadings>
double GradientDecentModeBASE<numVars, numReadings>::getAlphaReadingsEWMA()
{
	if (adaptiveAlphaReadingsEWMA)
	{
		double rotationRate = abs(_MPU9150->getRotationRate());
		if (rotationRate < 1.5)
			return DEFAULT_SLOW_READINGS_EWMA_ALPHA;
		else
			return DEFAULT_FAST_READINGS_EWMA_ALPHA;
	}
	else
	{
		return alphaReadingsEWMA;
	}
}

template <int numVars, int numReadings>
double GradientDecentModeBASE<numVars, numReadings>::RPSToMoveAmount()
{
	// 2rps: 14mm
	//  4rps: 8mm
	//  5rps:  5mm

	double currRotationRate = _MPU9150->getRotationRate();

	return -3 * currRotationRate + 20;
}

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::setState(ControlState newState)
{
	PositionControlMode::setState(newState);
	currAutoState = AutoState::waiting;
	// dataCollectionIteration = 0;
	pointQueue.clear();
	printedSurfaceState = false;
	// runningAveragesActivated = false;

	// yAmplitudeAveragesActivated = false;

#if USE_X_AND_Y_READINGS_FOR_ERROR
	// xAmplitudeAveragesActivated = false;
#endif
}

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::printHeader()
{
	if (printType == 'v')
	{

#if USE_X_AND_Y_READINGS_FOR_ERROR
		dualSerial.println("xPos\tyPos\tgx\tgxAvg\tgxAmAvg\tgxAmRate\tgy\tgyAvg\tgyAmAvg\tgyAmRate");
#else
		dualSerial.println("xPos\tyPos\tgy\tgyAvg\tgyAmAvg\tgyAmRate");
#endif
	}
	else
	{
		PositionControlMode::printHeader();
	}
}

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::printDeviceState()
{
	if (printType == 'v')
	{
		printMassPositions(false, false);
		dualSerial.print("\t");
#if USE_X_AND_Y_READINGS_FOR_ERROR
		dualSerial.print(_MPU9150->gx);
		dualSerial.print("\t");
		dualSerial.print(gxRunningAverage);
		dualSerial.print("\t");
		dualSerial.print(gxAmplitudeEWMA);
		dualSerial.print("\t");
		dualSerial.print(gxAmplitudeRateEWMA);
		dualSerial.print("\t");
#endif
		dualSerial.print(_MPU9150->gy);
		dualSerial.print("\t");
		dualSerial.print(gyRunningAverage);
		dualSerial.print("\t");
		dualSerial.print(gyAmplitudeEWMA);
		dualSerial.print("\t");
		dualSerial.print(gyAmplitudeRateEWMA);
		dualSerial.println();
	}
	else
	{
		PositionControlMode::printDeviceState();
	}
}

actionState GradientDecentModePoly22::goToNextPosition()
{
	if (!nextPostionFoundAndSet)
	{
		XYPoint localMinima = surface.getLocalMinima();
		_positionController->setDesiredXY(localMinima);
		nextPostionFoundAndSet = true;
	}

	if (_positionController->positionControl() == actionState::FINISHED)
	{
		nextPostionFoundAndSet = false;
		return actionState::FINISHED;
	}

	return actionState::RUNNING;
}

template <int numVars, int numReadings>
void GradientDecentModeBASE<numVars, numReadings>::printSurface()
{
	surface.printSurfaceCoefficients();
}

// POLY MODE DIFFERENCES

void GradientDecentModePoly22::queueMovePoints()
{
	double currX = 0;
	double currY = 0;
	if (_positionController->bothWeightsZeroed())
	{
		currX = _positionController->getXPosition();
		currY = _positionController->getYPosition();
	}

	// https://mycurvefit.com/ y = 5.784456 + 34.20486*e^(-0.896212*x)
	float moveAmount = 6 + 34 * exp(-1 * dataCollectionIteration);

	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount;
	currY -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currY += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currY += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currY -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
}

void GradientDecentModePoly21::queueMovePoints()
{
	double currX = _positionController->getXPosition();
	double currY = _positionController->getYPosition();

	// https://mycurvefit.com/ y = 5.784456 + 34.20486*e^(-0.896212*x)
	float moveAmount = 6 + 34 * exp(-1 * (dataCollectionIteration));

	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount / 2.0;
	currY -= moveAmount / 2.0;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currY += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
}

actionState GradientDecentModePoly21::performGradDecent()
{
	// Set the next collect data points.
	if (currAutoState == AutoState::waiting)
	{
		initialPosition = _positionController->getXYPosition();
		queueMovePoints();
		surface.reset();
		surfacePOLY12.reset();
		currAutoState = AutoState::collectingData;
	}

	if (currAutoState == AutoState::collectingData)
	{
		if (collectData() == actionState::FINISHED)
		{

			currAutoState = AutoState::movingToMinima;
		}
	}

	if (currAutoState == AutoState::movingToMinima)
	{

		if (!surface.operationCompleted || !surfacePOLY12.operationCompleted)
		{
			surface.performLeastSquaresOperation();
			surfacePOLY12.performLeastSquaresOperation();
			if (!printedSurfaceState && (!surface.validReductionOperation ||
										 !surface.validSurface ||
										 !surfacePOLY12.validReductionOperation ||
										 !surfacePOLY12.validSurface))
			{
				if (!surface.validReductionOperation)
					dualSerial.println("*ERROR* Invalid Row Echelon Operation on POLY21");
				if (!surface.validSurface)
				{
					dualSerial.println("*ERROR* Invalid Surface on POLY21");
					surface.printSurfaceCoefficients();
				}
				if (!surfacePOLY12.validReductionOperation)
					dualSerial.println("*ERROR* Invalid Row Echelon Operation on POLY12");
				if (!surfacePOLY12.validSurface)
				{
					dualSerial.println("*ERROR* Invalid Surface on POLY12");
					surfacePOLY12.printSurfaceCoefficients();
				}
			}

			dualSerial.println("Competed surface operation");
		}

		if ((surface.validReductionOperation &&
			 surface.validSurface))
		{
			if (!printedSurfaceState)
			{
				surface.printSurfaceCoefficients();
				printedSurfaceState = true;
			}

			if (goToNextPosition() == actionState::FINISHED)
				currAutoState = AutoState::finishedMoving;
		}
		else if (surfacePOLY12.validReductionOperation &&
				 surfacePOLY12.validSurface)
		{
			if (!printedSurfaceState)
			{
				surfacePOLY12.printSurfaceCoefficients();
				printedSurfaceState = true;
			}

			if (goToNextPosition() == actionState::FINISHED)
				currAutoState = AutoState::finishedMoving;
		}
		else
		{
			if (!returningToCentre)
			{
				dualSerial.println("Invalid surface, moving to the centre and trying again");
				returningToCentre = true;
			}

			_positionController->setDesiredX(0);
			_positionController->setDesiredY(0);

			if (_positionController->positionControl() == actionState::FINISHED)
			{
				returningToCentre = false;
				dataCollectionIteration--; // undo last iteration
				currAutoState = AutoState::waiting;
				printedSurfaceState = false;
			}
		}
	}

	// In the case that have moved to end, and still vibrating
	if (dataCollectionIteration > 0 && currAutoState == AutoState::finishedMoving)
	{
		// Finish even if not at minima
		printingEachItter = false;
		return actionState::FINISHED;

		if (getEWMAAmplitudeValue() > STABLE_AMPLITUDE_THRESHOLD)
		{
			currAutoState = AutoState::waiting;
		}
		else
		{
			return actionState::FINISHED;
		}
	}

	return actionState::RUNNING;
}

void GradientDecentModePoly21::storeCollectedData()
{
	XYPoint currPosition = _positionController->getXYPosition();
	surfacePOLY12.addReadings(currPosition.x, currPosition.y, stabilisingEWMAAmplitude, AMPLITUDE_READINGS_NORMALISATION_POWER);
	GradientDecentModeBASE::storeCollectedData();
}

actionState GradientDecentModePoly21::goToNextPosition()
{
	if (!nextPostionFoundAndSet)
	{
		XYPoint directionOfDecent(0, 0);
		if (surface.validSurface)
			directionOfDecent = surface.directionOfSteepestDecent(initialPosition);
		else
			directionOfDecent = surfacePOLY12.directionOfSteepestDecent(initialPosition);

		_positionController->setDesiredXY(directionOfDecent);
		nextPostionFoundAndSet = true;
	}

	if (_positionController->positionControl() == actionState::FINISHED)
	{
		nextPostionFoundAndSet = false;
		return actionState::FINISHED;
	}

	return actionState::RUNNING;
}

void GradientDecentModePoly11::queueMovePoints()
{
	double currX = _positionController->getXPosition();
	double currY = _positionController->getYPosition();

	// https://mycurvefit.com/ y = 5.784456 + 34.20486*e^(-0.896212*x)
	float moveAmount = 3 + 15 * exp(-1 * (dataCollectionIteration));

	pointQueue.enqueue(XYPoint(currX, currY));
	currX += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currY += moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
	currX -= moveAmount;
	pointQueue.enqueue(XYPoint(currX, currY));
}

void GradientDecentModePoly21::printSurface()
{
	dualSerial.println("Poly12 Surface:");
	surfacePOLY12.printSurfaceCoefficients();
	dualSerial.println("Poly21 Surface:");
	surface.printSurfaceCoefficients();
}
