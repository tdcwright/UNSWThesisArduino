#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>
#include "../../pinDefinitions.h"

#include "../dualSerial/dualSerial.h"
extern DualSerial dualSerial;

typedef byte encoderSelect;

typedef byte encoderNumber;
#define ENCODER_NONE 0
#define ENCODER_1 1 // TOP ENCODER
#define ENCODER_2 2 // BOTTOM ENCODER

#define ENCODER_1_READING_MULTIPLIER 4
#define ENCODER_2_READING_MULTIPLIER -1

enum CountMode
{
	CountMode_ClockDir = 0,
	CountMode_x1,
	CountMode_x2,
	CountMode_x4,
	CountMode_count
};
typedef enum CountMode CountMode_t;

class Encoder
{
public:
	Encoder(byte EN1_SSPin, byte EN1_ENPin, byte EN2_SSPin, byte EN2_ENPin); // Constructor to choose SS pin and enable pin
	void begin();

	// Definition of counting mode: clock/dir, x1, x2 or x4
	bool setCountMode(encoderNumber selectedEncoder, CountMode_t mode);

	// Enable counting
	void enable(encoderNumber selectedEncoder);

	// Stop counting
	void disable(encoderNumber selectedEncoder);

	// Zero the encoder at the current position
	void zeroEncoder(encoderNumber selectedEncoder);

	// Defines the new encoder position
	void setEncoder(encoderNumber selectedEncoder, const long newEncoderPosition);

	// Returns the current encoder position
	long getReading(encoderNumber selectedEncoder);

	// Top Weight Position in mm with the non-pulley mount face being 0
	double getXPosition();

	// Bottom Weight Position in mm with the non-pulley mount face being 0
	double getYPosition();

	uint16_t getXZeroCounter();
	uint16_t getYZeroCounter();

protected:
	long encoder1Position;
	long encoder2Position;
	byte ss1Pin;
	byte en1Pin;
	byte ss2Pin;
	byte en2Pin;

private:
	unsigned char mdr0, mdr1;

	encoderNumber currActiveEncoder;
	encoderSelect encoderSelectPin(encoderNumber selectedEncoder);
	bool selectEncoder(encoderNumber selectedEncoder);
	void writeToRegister0(encoderNumber selectedEncoder);
	void writeToRegister1(encoderNumber selectedEncoder);

	uint16_t xZeroCounter;
	uint16_t yZeroCounter;
};
#endif