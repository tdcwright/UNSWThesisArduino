#include "idleMode.h"

controllerMode IdleMode::detectCommand()
{
    if ((*_inputString)[_inputString->length() - 1] == '\n')
    {
        char command1 = (*_inputString)[0];
        char command2 = (*_inputString)[1];

        switch (command1)
        {
        case 'a':
            switch (command2)
            {
            case 'g':
                switch ((*_inputString)[2])
                {
                case '2':
                    return controllerMode::GRADIENT_DECENT_POLY22;
                    break;
                case '1':
                    return controllerMode::GRADIENT_DECENT_POLY21;
                    break;
                case '0':
                    return controllerMode::GRADIENT_DECENT_POLY11;
                    break;

                default:
                    dualSerial.println("No gradient decent value matches input.");
                    break;
                }
                break;
            case 's':
                return controllerMode::SLIDING_MODE;
                break;
            default:
                dualSerial.println("Invalid automatic controller mode");
                break;
            }
            break;
        case 'p':
            return controllerMode::POSITION_CONTROL;
            break;
        case 'g':
            return controllerMode::ACCELEROMETER;
            break;
        case 'm':
            return controllerMode::MOTOR_CONTROL;
            break;
        case 'l':
            return controllerMode::LIMIT_SWITCH;
            break;
        case 'c':
            return controllerMode::ENCODER;
            break;
        case 'h':
        default:
            printHelp();
            return controllerMode::NO_CHANGE;
            break;
        }
    }

    return controllerMode::NO_CHANGE;
}

modeResult IdleMode::runMode()
{
    controllerMode newMode = detectCommand();

    if (newMode == controllerMode::NO_CHANGE)
    {
        return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
    }

    return modeResult(runResult::MODE_CHANGE, newMode);
}

void IdleMode::printHelp()
{
    dualSerial.println("Help in mode: IDLE. Only the first letter command will be accepted");
    dualSerial.println("\th: Help");
    dualSerial.println("\tag[2/1/0]: Change to Gradient Decent Mode. 2: Poly22, 1: Poly21, 0: Poly11");
    dualSerial.println("\tas: Change to Sliding mode control mode");
    dualSerial.println("\tp: Change to Position Control Mode");
    dualSerial.println("\tg: Change to Gyro/Accelerometer Mode");
    dualSerial.println("\tm: Change to Motor Mode");
    dualSerial.println("\tc: Change to Encoder Mode");
    dualSerial.println("\tl: Change to Limit Switch Mode");
    dualSerial.println("\te: in any mode, return to IDLE");
}
