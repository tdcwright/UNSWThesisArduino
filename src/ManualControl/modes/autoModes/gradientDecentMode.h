#ifndef GRADIENT_DECENT_MODE_H
#define GRADIENT_DECENT_MODE_H
#include <Arduino.h>

#include "../../../Accelerometer/accelerometer.h"
#include "../../../Encoder/encoder.h"
#include "../../../positionController/positionController.h"

#include "../../../dualSerial/dualSerial.h"

#include "../baseMode.h"
#include "positionControlMode.h"

#include "../../../helpers/circularArray.h"
#include "../../../helpers/Surfaces/Surfaces.h"
#include "../../../helpers/MovePoints.h"

extern DualSerial dualSerial;

#define DEFAULT_SLOW_READINGS_EWMA_ALPHA 0.05
#define DEFAULT_FAST_READINGS_EWMA_ALPHA 0.01
#define DEFAULT_AMPLITUDE_EWMA_ALPHA 0.3
#define DEFAULT_AMPLITUDE_RATE_EWMA_ALPHA 0.5
#define STABILISING_EWMA_AMPLITUDE_ALPHA 0.3

#define NUM_REV_UNTIL_STABLE 5

#define NUMBER_OF_PAST_GYRO_VALUES 50

#define USE_CROSSED_AVERAGE false		   // decides whether to use timed wavelength or crossed average wavelength
#define NUM_OF_CHECKBACK_VALUES 3		   // NOT USED IF USE_CROSSED_AVERAGE IS FALSE. number of values to check ago to see whether the readings have crossed the average
#define MILLIS_THRESHOLD_FOR_RPS_AVERAGE 5 // Subtraction value from measured millis to account for rps innacuracy

#define AMPLITUDE_READINGS_NORMALISATION_POWER 1 // amplitude reading to the power of X to increase the readings for surface fitting
#define STABLE_AMPLITUDE_THRESHOLD 0.5			 // Minimum level of vibration to continue finding minimum
#define AMPLITUDE_STABLE_PAST_THRESHOLD 0.5

#define USE_X_AND_Y_READINGS_FOR_ERROR false

enum class AutoState
{
	waiting,
	collectingData,
	movingToMinima,
	moving, // for sliding mode control
	finishedMoving
};

class GradientDecentModeBASE : public PositionControlMode
{
protected:
	surfaceFitType fitType;
	SurfaceData surface;
	AutoState currAutoState;
	PointQueue pointQueue;
	int dataCollectionIteration;
	bool returningToCentre;

	bool printedSurfaceState;

	bool adaptiveAlphaReadingsEWMA;
	double alphaReadingsEWMA; // Alpha for readings EWMA
	double getAlphaReadingsEWMA();
	bool runningAveragesActivated;

	float getEWMAAmplitudeValue();
	float getEWMAAmplitudeRateValue();
	unsigned long pastTime;
	unsigned long wavelengthTime;
	unsigned long numberOfWavelengths;
	double alphaAmplitudeEWMA; // Alpha for amplitude EWMA

	float gyRunningAverage; // Exponentially weighted moving average (EWMA)
	bool yAmplitudeAveragesActivated;
	float gyLastAmplitudeEWMA;
	float gyAmplitudeRateEWMA;
	float gyAmplitudeEWMA;
	CircularArray<float> pastGY; // used to find min and max in a wavelength

#if USE_X_AND_Y_READINGS_FOR_ERROR
	float gxRunningAverage; // Exponentially weighted moving average (EWMA)
	bool xAmplitudeAveragesActivated;
	float gxLastAmplitudeEWMA;
	float gxAmplitudeRateEWMA;
	float gxAmplitudeEWMA;
	CircularArray<float> pastGX; // used to find min and max in a wavelength
#endif

	void collectReadings();
	void collectReadingsSingleAxis(float newReading, CircularArray<float> &pastReadings, float &runningAverage, bool &amplitudeAvgSet, float &amplitudeEWMA, float &lastAmplitudeEWMA, float &amplitudeRateEWMA, bool &newWavelength);
	bool checkbackCrossedAverage(CircularArray<float> &pastReadings, float &averageReadings);

	// Runs gradient decent operation. RUNNING, FINISHED
	actionState performGradDecent();
	// Moves to positions. RUNNING, WAITING, FINISHED
	actionState collectData();
	// Returns whether the gradient of the amplitude is low
	bool isAmplitudeStable();
	unsigned long initialNumWavelengths;
	bool initialNumWavelengthsFound;
	float stabilisingEWMAAmplitude; // Updated when waiting for readings to be stable

	bool nextPostionFoundAndSet; // Ensures next position is only calculated once
	// Performs gradient decent on the collected data. FINISHED, RUNNING. By default goes down steepest decent
	virtual actionState goToNextPosition();
	// Adds a series of points to the move queue, centered around the current location;
	virtual void queueMovePoints() = 0;

	controllerMode detectCommand();

public:
	GradientDecentModeBASE(String *inputString, PositionController *positionController, Accelerometer *MPU9150, surfaceFitType sFitType);
	virtual ~GradientDecentModeBASE(){};

	virtual void setState(ControlState newState);
	void printHeader();
	void printDeviceState();
	void printSurfaceCoefficients();

	void printHelp();
	modeResult runMode();
};

// Tries to find local minima using poly22 function
class GradientDecentModePoly22 : public GradientDecentModeBASE
{
protected:
	// Performs gradient decent on the collected data. FINISHED, RUNNING
	actionState goToNextPosition();
	// Adds a series of points to the move queue, centered around the current location;
	void queueMovePoints();

public:
	GradientDecentModePoly22(String *inputString, PositionController *positionController, Accelerometer *MPU9150) : GradientDecentModeBASE(inputString, positionController, MPU9150, surfaceFitType::poly22){};
	~GradientDecentModePoly22(){};

	void setState(ControlState newState) { GradientDecentModeBASE::setState(newState); };
};

// ****************
// Tries to find local minima using poly21 function
class GradientDecentModePoly21 : public GradientDecentModeBASE
{
protected:
	// Performs gradient decent on the collected data. FINISHED, RUNNING
	actionState goToNextPosition() { GradientDecentModeBASE::goToNextPosition(); };
	// Adds a series of points to the move queue, centered around the current location;
	void queueMovePoints();

public:
	GradientDecentModePoly21(String *inputString, PositionController *positionController, Accelerometer *MPU9150) : GradientDecentModeBASE(inputString, positionController, MPU9150, surfaceFitType::poly21){};
	~GradientDecentModePoly21(){};
	void setState(ControlState newState) { GradientDecentModeBASE::setState(newState); };
};

// Same as GradientDecentModePoly21 Tries to find local minima using poly21 function
class GradientDecentModePoly12 : public GradientDecentModePoly21
{
public:
	GradientDecentModePoly12(String *inputString, PositionController *positionController, Accelerometer *MPU9150) : GradientDecentModePoly21(inputString, positionController, MPU9150){};
	~GradientDecentModePoly12(){};
};

// Tries to find local minima using poly21 function
class GradientDecentModePoly11 : public GradientDecentModeBASE
{
protected:
	// Performs gradient decent on the collected data. FINISHED, RUNNING
	actionState goToNextPosition() { GradientDecentModeBASE::goToNextPosition(); };
	// Adds a series of points to the move queue, centered around the current location;
	void queueMovePoints();

public:
	GradientDecentModePoly11(String *inputString, PositionController *positionController, Accelerometer *MPU9150) : GradientDecentModeBASE(inputString, positionController, MPU9150, surfaceFitType::poly11){};
	~GradientDecentModePoly11(){};

	void setState(ControlState newState) { GradientDecentModeBASE::setState(newState); };
};

#endif