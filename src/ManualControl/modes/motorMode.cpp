#include "motorMode.h"

controllerMode MotorMode::detectCommand(){
    if ((*_inputString)[_inputString->length()-1]=='\n'){
        char command = (*_inputString)[0];

        switch (command){
            case 's':
            case 'f':
            case 'r':
            case 'x':
            case 'c':
            case 'y':
            case 'u':
                runAction(command);
                lastAction = command;
                break;
            case 'q':
                increaseSpeed(10);
                dualSerial.print("New Set Speed: ");dualSerial.print(currSpeed);dualSerial.println("%");
                runAction(lastAction);
                break;
            case 'a':
                decreaseSpeed(10);
                dualSerial.print("New Set Speed: ");dualSerial.print(currSpeed);dualSerial.println("%");
                runAction(lastAction);
                break;
			case 'e':
                _motorController->stopMotor(MOTOR_BOTH);
				return controllerMode::IDLE;
            case 'h':
            default:
                printHelp();
                break;
        }
    }
    
    return controllerMode::NO_CHANGE;
}

void MotorMode::runAction(char actionCode){
    switch (actionCode){
        case 's':
            _motorController->stopMotor(MOTOR_BOTH);
            //dualSerial.println("STOPPING MOTOR");
            break;
        case 'f':
            //dualSerial.print("Forward at speed ");dualSerial.println(currSpeed);
            _motorController->moveMotors(MOTOR_BOTH, motorDirection::forward, currSpeed);
            break;
        case 'r':
            //dualSerial.print("Reverse at speed ");dualSerial.println(currSpeed);
            _motorController->moveMotors(MOTOR_BOTH, motorDirection::backwards, currSpeed);
            break;
        case 'x':
            //dualSerial.print("X Forward at speed ");dualSerial.println(currSpeed);
            _motorController->moveX(axisDirection::positive, currSpeed);
            break;
        case 'c':
            //dualSerial.print("X Reverse at speed ");dualSerial.println(currSpeed);
            _motorController->moveX(axisDirection::negative, currSpeed);
            break;
        case 'y':
            //dualSerial.print("Y Forward at speed ");dualSerial.println(currSpeed);
            _motorController->moveY(axisDirection::positive, currSpeed);
            break;
        case 'u':
            //dualSerial.print("Y Reverse at speed ");dualSerial.println(currSpeed);
            _motorController->moveY(axisDirection::negative, currSpeed);
            break;
    }
}

modeResult MotorMode::runMode(){
    controllerMode newMode = detectCommand();

    if (newMode != controllerMode::NO_CHANGE){
		return modeResult(runResult::MODE_CHANGE, newMode);
    }

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}


void MotorMode::printHelp(){
    dualSerial.println("Help in mode: MOTOR CONTROL. Only the first letter command will be accepted");
    dualSerial.println("\th: Help");
    dualSerial.println("\ts: stop all");
    dualSerial.println("\tf: forward both");
    dualSerial.println("\tr: reverse both");
    dualSerial.println("\tx: forward X");
    dualSerial.println("\tc: reverse X");
    dualSerial.println("\ty: forward Y");
    dualSerial.println("\tu: reverse Y");
    dualSerial.print("\tSpeed Control, currently at ");dualSerial.print(currSpeed);dualSerial.println("%");
    dualSerial.print("\t\tq: Speed up 10%");
    dualSerial.println("\t\ta: Speed down 10%");
    dualSerial.println("\te: return to IDLE");
}


void MotorMode::increaseSpeed(uint8_t ammount){
    currSpeed += ammount;

    if (currSpeed > SPEED_FULL) currSpeed = SPEED_FULL;
}

void MotorMode::decreaseSpeed(uint8_t ammount){
    currSpeed -= ammount;

    if (currSpeed < SPEED_STOP) currSpeed = SPEED_STOP;
}