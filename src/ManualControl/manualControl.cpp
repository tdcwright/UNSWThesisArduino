#include "manualControl.h"

#include "modes/idleMode.h"
#include "modes/autoModes/BroydenFunctionMode.h"
#include "modes/autoModes/slidingModeControlMode.h"
#include "modes/autoModes/gradientDecentMode.h"
#include "modes/autoModes/positionControlMode.h"
#include "modes/limitSwtichMode.h"
#include "modes/motorMode.h"
#include "modes/accelerometerMode.h"
#include "modes/encoderMode.h"

ManualController::ManualController(PositionController *positionController, MotorController *motorController, Accelerometer *MPU9150, Encoder *encoder, LimitSwitch *LSYellow, LimitSwitch *LSPink, LimitSwitch *LSGrey, LimitSwitch *LSWhite)
{
	_positionController = positionController;
	_motorController = motorController;

	_MPU9150 = MPU9150;

	_encoder = encoder;

	_LSYellow = LSYellow;
	_LSPink = LSPink;
	_LSGrey = LSGrey;
	_LSWhite = LSWhite;
}

void ManualController::begin()
{
	setMode(controllerMode::IDLE, false);
}

void ManualController::getInput()
{
	if (dualSerial.available())
	{
		inputString = dualSerial.readString();
	}
	else
	{
		inputString = "";
	}
}

void ManualController::setMode(controllerMode newMode, bool deletePrevious)
{
	currMode = newMode;
	if (deletePrevious)
		delete modeController;
	switch (newMode)
	{
	case controllerMode::GRADIENT_DECENT_POLY22:
		modeController = new GradientDecentModePoly22(&inputString, _positionController, _MPU9150);
		break;
	case controllerMode::GRADIENT_DECENT_POLY21:
		modeController = new GradientDecentModePoly21(&inputString, _positionController, _MPU9150);
		break;
	case controllerMode::GRADIENT_DECENT_POLY11:
		modeController = new GradientDecentModePoly11(&inputString, _positionController, _MPU9150);
		break;
	case controllerMode::SLIDING_MODE:
		modeController = new SlidingModeControlMode(&inputString, _positionController, _MPU9150);
		break;
	case controllerMode::BROYDEN_MODE:
		modeController = new BroydenFunctionMode(&inputString, _positionController, _MPU9150);
		break;
	case controllerMode::POSITION_CONTROL:
		modeController = new PositionControlMode(&inputString, _positionController, _MPU9150, _encoder);
		break;
	case controllerMode::LIMIT_SWITCH:
		modeController = new LimitSwitchMode(&inputString, _LSYellow, _LSPink, _LSGrey, _LSWhite);
		break;
	case controllerMode::MOTOR_CONTROL:
		modeController = new MotorMode(&inputString, _motorController);
		break;
	case controllerMode::ENCODER:
		modeController = new EncoderMode(&inputString, _encoder);
		break;
	case controllerMode::ACCELEROMETER:
		modeController = new AccelerometerMode(&inputString, _MPU9150);
		break;
	case controllerMode::IDLE:
	default:
		modeController = new IdleMode(&inputString);
		break;
	}

	modeController->printHelp();
	delay(100);
}

void ManualController::runController()
{
	getInput();

	modeResult latestResult = modeController->runMode();

	if (latestResult.currResult == runResult::MODE_CHANGE)
	{
		setMode(latestResult.newMode);
	}
}