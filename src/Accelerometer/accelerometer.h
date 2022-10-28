#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H
#include <Arduino.h>

#include "../dualSerial/dualSerial.h"

#include "../helpers/circularArray.h"

extern DualSerial dualSerial;

#define INCLUDE_MAG_READINGS false

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "MPU9150Firmware/I2Cdev/I2Cdev.h"
#include "MPU9150Firmware/MPU6050/MPU6050.h"

#define CENTRE_TO_SENSOR_DISTANCE 58.582f
#define RADS_TO_REVS 0.159f
#define G_TO_MSS 9.8f
#define AX_TO_A_ON_R 0.167f // G_TO_MSS/CENTRE_TO_SENSOR_DISTANCE

typedef bool zeroMode;
#define ACCEL_ZERO_VALS true
#define ACCEL_RESET_ZEROS false

#define ACCEL_READ_RANGE_MODE 1
#if ACCEL_READ_RANGE_MODE == 0
#define MAX_ACCEL_READING 2.0f
#elif ACCEL_READ_RANGE_MODE == 1
#define MAX_ACCEL_READING 4.0f
#elif ACCEL_READ_RANGE_MODE == 2
#define MAX_ACCEL_READING 8.0f
#elif ACCEL_READ_RANGE_MODE == 3
#define MAX_ACCEL_READING 16.0f
#endif

#define GYRO_READ_RANGE_MODE 3
#if GYRO_READ_RANGE_MODE == 0
#define MAX_GYRO_READING 250.0f
#define MAX_GYRO_READING_TRESHOLD 2
#elif GYRO_READ_RANGE_MODE == 1
#define MAX_GYRO_READING 500.0f
#define MAX_GYRO_READING_TRESHOLD 3
#elif GYRO_READ_RANGE_MODE == 2
#define MAX_GYRO_READING_TRESHOLD 5
#define MAX_GYRO_READING 1000.0f
#define MAX_GYRO_READING_TRESHOLD 5
#elif GYRO_READ_RANGE_MODE == 3
#define MAX_GYRO_READING 2000.0f
#define MAX_GYRO_READING_TRESHOLD 10
#endif

#define LAST_VALUES_COUNT 30 // used only for ax values in this class

extern MPU6050 MPUDevice;

class Accelerometer
{

private:
  byte interruptPin;
  bool successfulConnection;

  double maxAccelReading();
  double maxGyroReading();

  CircularArray<float> pastAX;

  int16_t a1, a2, a3, g1, g2, g3; // raw data arrays reading

#if INCLUDE_MAG_READINGS
  int16_t m1, m2, m3; // raw data arrays reading
#endif

public:
  Accelerometer(byte intPin);
  void begin();

  void disable();
  void enable();
  bool dataReady();
  void getData();

  void printHeader();
  void printData();
  void printAccel(bool printEndLine = false);
  void printGyro(bool printEndLine = false);

  void zeroAll(zeroMode action);
  void zeroAccel(zeroMode action);
  void zeroGyro(zeroMode action);
#if INCLUDE_MAG_READINGS
  void zeroMag(zeroMode action);
#endif

  // Rev/s. Uses z-gyroscope reading to calculate rotational rate. If gyro is at limit, then use x-accelerometer (less accurage)
  double getRotationRate();

  unsigned long collectedTime;
  float ax, ay, az, gx, gy, gz;                               // variables to hold latest sensor data values
  float zero_ax, zero_ay, zero_az, zero_gx, zero_gy, zero_gz; // variables to hold latest sensor data values

#if INCLUDE_MAG_READINGS
  float mx, my, mz;                // variables to hold latest sensor data values
  float zero_mx, zero_my, zero_mz; // variables to hold latest sensor data values
#endif
};

#endif