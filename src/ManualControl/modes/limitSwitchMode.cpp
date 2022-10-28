#include "limitSwtichMode.h"

controllerMode LimitSwitchMode::detectCommand(){
    if ((*_inputString)[_inputString->length()-1]=='\n'){
        char command = (*_inputString)[0];

        switch (command){
            case 'w':
                limitSwitchChosen = true;
				activeLimitSwitch = _LSWhite;
                break;
            case 'g':
                limitSwitchChosen = true;
				activeLimitSwitch = _LSGrey;
                break;
            case 'p':
                limitSwitchChosen = true;
				activeLimitSwitch = _LSPink;
                break;
            case 'y':
                limitSwitchChosen = true;
				activeLimitSwitch = _LSYellow;
                break;
			case 's':
                printStateToggle = !printStateToggle;
               dualSerial.print("Limit SwitchdualSerial Monitor is ");dualSerial.println(printStateToggle?"*ACTIVE*":"*DISABLED*");
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

modeResult LimitSwitchMode::runMode(){
    controllerMode newMode = detectCommand();

    if (newMode != controllerMode::NO_CHANGE){
		return modeResult(runResult::MODE_CHANGE, newMode);
    }

	if (limitSwitchChosen && activeLimitSwitch->isPressed())
	{
		if (printStateToggle){
            dualSerial.print(activeLimitSwitch->switchName);
            dualSerial.println(" Switch is: Active");
        }
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	}
	else
	{
		if (limitSwitchChosen && printStateToggle){
            dualSerial.print(activeLimitSwitch->switchName);
            dualSerial.println(" Switch is: Inactive");
        }
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
	}

	return modeResult(runResult::MODE_SUCESS, controllerMode::NO_CHANGE);
}


void LimitSwitchMode::printHelp(){
dualSerial.println("Help in mode: LIMIT SWITCH. Only the first letter command will be accepted");
dualSerial.println("\th: Help");
dualSerial.println("\tw: Link White Limit Switch to LED");
dualSerial.println("\tg: Link Grey Limit Switch to LED");
dualSerial.println("\tp: Link Pink Limit Switch to LED");
dualSerial.println("\ty: Link Yellow Limit Switch to LED");
dualSerial.print("\ts: Toggle printing switch state to Serial. Current State is: ");dualSerial.println(printStateToggle?"TRUE":"FALSE");
dualSerial.println("\te: return to IDLE");
}