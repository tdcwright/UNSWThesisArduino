#include "encoderMode.h"

controllerMode EncoderMode::detectCommand(){
    if ((*_inputString)[_inputString->length()-1]=='\n'){
        char command = (*_inputString)[0];

        switch (command){
            case 'b':
                currentPrintMode = PRINT_ENCODER_BOTH;
                break;
            case '1':
                currentPrintMode = PRINT_ENCODER_1;
                break;
            case '2':
                currentPrintMode = PRINT_ENCODER_2;
                break;
            case 's':
                currentPrintMode = PRINT_ENCODER_NONE;
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

void EncoderMode::zeroReadings(){
    if (_inputString->length()-1 == 2){
        char zeroAction = (*_inputString)[1];
        switch (zeroAction)
        {
        case 'b':
            _encoder->zeroEncoder(ENCODER_1);
            _encoder->zeroEncoder(ENCODER_2);
            break;
        case '1':
            _encoder->zeroEncoder(ENCODER_1);
            break;
        case '2':
            _encoder->zeroEncoder(ENCODER_2);
            break;
        }
    } else {
       dualSerial.println("Incorrect string format for zeroing readings");
    }
}

modeResult EncoderMode::runMode(){
    controllerMode newMode = detectCommand();

    if (newMode != controllerMode::NO_CHANGE){
		return modeResult(runResult::MODE_CHANGE, newMode);
    }

    printReadings();

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}

void EncoderMode::printReadings(){
    switch (currentPrintMode)
    {
    case PRINT_ENCODER_1:
       dualSerial.print("E1:\t");dualSerial.println(_encoder->getReading(ENCODER_1));
        break;
    case PRINT_ENCODER_2:
       dualSerial.print("E2:\t");dualSerial.println(_encoder->getReading(ENCODER_2));
        break;
    case PRINT_ENCODER_BOTH:
       dualSerial.print("E1:\t");dualSerial.print(_encoder->getReading(ENCODER_1));
       dualSerial.print("\t|\t");
       dualSerial.print("E2:\t");dualSerial.println(_encoder->getReading(ENCODER_2));
        break;
    }
}

void EncoderMode::printHelp(){
	dualSerial.println("Help in mode: ENCODER. Only the first letter command will be accepted");
	dualSerial.println("\th: Help");
	dualSerial.println("\t1: Print Encoder 1 value");
	dualSerial.println("\t2: Print Encoder 2 value");
	dualSerial.println("\tb: Print both Encoder values");
	dualSerial.println("\ts: Stop printing readings");
	dualSerial.println("\tz1: Zero Encoder 1");
	dualSerial.println("\tz2: Zero Encoder 2");
	dualSerial.println("\tzb: Zero both encoders");
	dualSerial.println("\te: return to IDLE");
}