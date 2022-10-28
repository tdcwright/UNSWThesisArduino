#include "encoder.h"
#include "encoderCommandDefinitions.h"
#include "../helpers/encoderToMassPosition.h"

#include <SPI.h>
#include <Arduino.h>

Encoder::Encoder(byte EN1_SSPin, byte EN1_ENPin, byte EN2_SSPin, byte EN2_ENPin)
{
	ss1Pin = EN1_SSPin;
	en1Pin = EN1_ENPin;
	ss2Pin = EN2_SSPin;
	en2Pin = EN2_ENPin;

	encoder1Position = 0;
	encoder2Position = 0;

	currActiveEncoder = ENCODER_NONE;

	xZeroCounter = 0;
	yZeroCounter = 0;
}

void Encoder::begin()
{

	pinMode(ss1Pin, OUTPUT);
	pinMode(en1Pin, OUTPUT);
	pinMode(ss2Pin, OUTPUT);
	pinMode(en2Pin, OUTPUT);

	digitalWrite(en1Pin, HIGH);
	digitalWrite(en2Pin, HIGH);

	SPI.begin();

	mdr0 = MDR0_INITIAL_VALUE,
	mdr1 = MDR1_INITIAL_VALUE;

	dualSerial.println("Initialising Encoders");
	writeToRegister0(ENCODER_1);
	writeToRegister1(ENCODER_1);
	writeToRegister0(ENCODER_2);
	writeToRegister1(ENCODER_2);
	dualSerial.println("Encoder Initialisation Complete");

	enable(ENCODER_1);
	enable(ENCODER_2);
}

encoderSelect Encoder::encoderSelectPin(encoderNumber selectedEncoder)
{
	if (selectedEncoder == ENCODER_1)
	{
		return ss1Pin;
	}

	if (selectedEncoder == ENCODER_2)
	{
		return ss2Pin;
	}

	return -1;
}

bool Encoder::selectEncoder(encoderNumber selectedEncoder)
{
	if (selectedEncoder == ENCODER_1)
	{
		digitalWrite(encoderSelectPin(ENCODER_2), HIGH);
		digitalWrite(encoderSelectPin(ENCODER_1), LOW);
		return false;
	}

	if (selectedEncoder == ENCODER_2)
	{
		digitalWrite(encoderSelectPin(ENCODER_1), HIGH);
		digitalWrite(encoderSelectPin(ENCODER_2), LOW);
		return false;
	}

	if (selectedEncoder == ENCODER_NONE)
	{
		digitalWrite(encoderSelectPin(ENCODER_2), HIGH);
		digitalWrite(encoderSelectPin(ENCODER_1), HIGH);
		return false;
	}

	return true;
}

void Encoder::writeToRegister0(encoderNumber selectedEncoder)
{
	if (selectEncoder(selectedEncoder))
		return;

	SPI.transfer(OP_WRITE | REG_MDR0);
	SPI.transfer(mdr0);

	selectEncoder(ENCODER_NONE);
}

void Encoder::writeToRegister1(encoderNumber selectedEncoder)
{
	if (selectEncoder(selectedEncoder))
		return;

	SPI.transfer(OP_WRITE | REG_MDR1);
	SPI.transfer(mdr1);

	selectEncoder(ENCODER_NONE);
}

bool Encoder::setCountMode(encoderNumber selectedEncoder, CountMode_t mode)
{
	if ((mode < 0) || (mode >= CountMode_count))
		return false;

	mdr0 = (mdr0 & 0b11111100) | mode;

	writeToRegister0(selectedEncoder);

	return true;
}

void Encoder::enable(encoderNumber selectedEncoder)
{
	mdr1 = (mdr1 & 0b11111011); // MDR1_B2_ENABLE_COUNTING

	writeToRegister1(selectedEncoder);
}

void Encoder::disable(encoderNumber selectedEncoder)
{
	mdr1 = (mdr1 | 0b00000100); // MDR1_B2_DISABLE_COUNTING

	writeToRegister1(selectedEncoder);
}

void Encoder::zeroEncoder(encoderNumber selectedEncoder)
{
	setEncoder(selectedEncoder, getReading(selectedEncoder));
}

void Encoder::setEncoder(encoderNumber selectedEncoder, const long newEncoderPosition)
{
	if (selectEncoder(selectedEncoder))
		return;
	SPI.transfer(OP_CLEAR | REG_CNTR);
	selectEncoder(ENCODER_NONE);

	switch (selectedEncoder)
	{
	case ENCODER_1:
		encoder1Position = newEncoderPosition;
		xZeroCounter++;
		break;
	case ENCODER_2:
		encoder2Position = newEncoderPosition;
		yZeroCounter++;
		break;
	}
}

long Encoder::getReading(encoderNumber selectedEncoder)
{
	long regOTR;

	if (selectEncoder(selectedEncoder))
		return -1;
	SPI.transfer(OP_LOAD | REG_OTR);
	selectEncoder(ENCODER_NONE);

	if (selectEncoder(selectedEncoder))
		return -1;
	SPI.transfer(OP_READ | REG_OTR);
	regOTR = SPI.transfer(0x00);
	regOTR <<= 8;
	regOTR |= SPI.transfer(0x00);
	regOTR <<= 8;
	regOTR |= SPI.transfer(0x00);
	regOTR <<= 8;
	regOTR |= SPI.transfer(0x00);
	selectEncoder(ENCODER_NONE);

	switch (selectedEncoder)
	{
	case ENCODER_1:
		return (long)(regOTR)*ENCODER_1_READING_MULTIPLIER;
	case ENCODER_2:
		return (long)(regOTR)*ENCODER_2_READING_MULTIPLIER;
	}

	return -1;
}

double Encoder::getXPosition()
{
	return readingToPosition(getReading(ENCODER_1));
}

double Encoder::getYPosition()
{
	return readingToPosition(getReading(ENCODER_2));
}

uint16_t Encoder::getXZeroCounter()
{
	return xZeroCounter;
}

uint16_t Encoder::getYZeroCounter()
{
	return yZeroCounter;
}