#include "DiskArduino.h"

LimitSwitch LSYellow("Yellow", LS_PIN_YELLOW);
LimitSwitch LSPink("Pink", LS_PIN_PINK);
LimitSwitch LSGrey("Grey", LS_PIN_GREY);
LimitSwitch LSWhite("White", LS_PIN_WHITE);

MPU6050 MPUDevice;
Accelerometer MPU9150(ACC_INT_PIN);

MotorController motorController(MC_A_PWM, MC_B_PWM, MC_FORWARD_DIR, MC_REVERSE_DIR);

Encoder encoderCounters(CC_BUS_1_SELECT, CC_BUS_1_ENABLE, CC_BUS_2_SELECT, CC_BUS_2_ENABLE);

Bluetooth bluetoothChip(BT_EN_PIN, BT_STATE_PIN);

PositionController positionController(&motorController, &encoderCounters, &LSYellow, &LSPink, &LSGrey, &LSWhite);

ManualController manualControl(&positionController,
                               &motorController,
                               &MPU9150,
                               &encoderCounters,
                               &LSYellow,
                               &LSPink,
                               &LSGrey,
                               &LSWhite);

DualSerial dualSerial(&Serial, &Serial1);

unsigned long heartbeatTime = 0;
bool currHeatbeatState = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  dualSerial.begin();

  Wire.begin();
  MPU9150.begin();
  motorController.begin();
  encoderCounters.begin();

  LSYellow.begin();
  LSPink.begin();
  LSGrey.begin();
  LSWhite.begin();

  bluetoothChip.begin();

  manualControl.begin();
}

void loop()
{
  manualControl.runController();

  unsigned long currTime = millis();
  if (currTime > heartbeatTime + 1000)
  {
    heartbeatTime = currTime;
    currHeatbeatState = !currHeatbeatState;
    digitalWrite(LED_BUILTIN, currHeatbeatState ? HIGH : LOW);
  }

  delay(10);
}