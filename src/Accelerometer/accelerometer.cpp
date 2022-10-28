#include "accelerometer.h"

Accelerometer::Accelerometer(byte intPin) : pastAX(LAST_VALUES_COUNT)
{

    interruptPin = intPin;
    a1 = 0;
    a2 = 0;
    a3 = 0;
    g1 = 0;
    g2 = 0;
    g3 = 0;

    ax = 0;
    zero_ax = 0;
    ay = 0;
    zero_ay = 0;
    az = 0;
    zero_az = 0;
    gx = 0;
    zero_gx = 0;
    gy = 0;
    zero_gy = 0;
    gz = 0;
    zero_gz = 0;

    collectedTime = 0;

#if INCLUDE_MAG_READINGS
    m1 = 0;
    m2 = 0;
    m3 = 0;
    mx = 0;
    zero_mx = 0;
    my = 0;
    zero_my = 0;
    mz = 0;
    zero_mz = 0;
#endif
}

void Accelerometer::begin()
{
    pinMode(interruptPin, OUTPUT);

    dualSerial.println("Initializing I2C devices...");
    MPUDevice.initialize();

    // verify connection
    dualSerial.println("Testing device connections...");
    if (MPUDevice.testConnection())
    {
        successfulConnection = true;
        dualSerial.println("MPU9150 connection successful");

        // Full-scale range of the gyro sensors:
        // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
        MPUDevice.setFullScaleGyroRange(GYRO_READ_RANGE_MODE); // set gyro range to 250 degrees/sec

        // Full-scale accelerometer range.
        // The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
        MPUDevice.setFullScaleAccelRange(ACCEL_READ_RANGE_MODE); // set accelerometer to 2 g range

        dualSerial.println("Accelerometer Ranges set");

        MPUDevice.setIntDataReadyEnabled(true); // enable data ready interrupt

        dualSerial.println("Zeroing gyroscope. Please keep disk still.");
        if (dataReady())
        {
            getData();
            zeroGyro(ACCEL_ZERO_VALS);
            dualSerial.println("Successfully Zeroed gyroscope.");
        }
    }
    else
    {
        successfulConnection = false;
        dualSerial.println("MPU6050 connection FAILED");
    }
}

bool Accelerometer::dataReady()
{
    if (!successfulConnection)
        return false;
    return MPUDevice.getIntDataReadyStatus() == 1;
}

void Accelerometer::getData()
{
    if (!successfulConnection)
    {
        dualSerial.println("NO MPU9150 CONNECTION");
        ax = 0;
        ay = 0;
        az = 0;
        gx = 0;
        gy = 0;
        gz = 0;
#if INCLUDE_MAG_READINGS
        mx = 0;
        my = 0;
        mz = 0;
#endif

        return;
    }
    MPUDevice.getAcceleration(&a1, &a2, &a3);
    MPUDevice.getRotation(&g1, &g2, &g3);
    collectedTime = millis();

    ax = a1 * MAX_ACCEL_READING / 32768.0f - zero_ax; // 2 g full range for accelerometer
    ay = a2 * MAX_ACCEL_READING / 32768.0f - zero_ay;
    az = a3 * MAX_ACCEL_READING / 32768.0f - zero_az;

    // add ax to past readings
    pastAX.addValue(ax);

    gx = g1 * MAX_GYRO_READING / 32768.0f - zero_gx; // 250 deg/s full range for gyroscope
    gy = g2 * MAX_GYRO_READING / 32768.0f - zero_gy;
    gz = g3 * MAX_GYRO_READING / 32768.0f - zero_gz;
    //  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
    //  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
    //  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
    //  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and
    //  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
    //  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
    //  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched
    //  compared to the gyro and accelerometer!

#if INCLUDE_MAG_READINGS
    MPUDevice.getMag(&m1, &m2, &m3);
    mx = m1 * 10.0f * 1229.0f / 4096.0f + 18.0f - zero_mx; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
    my = m2 * 10.0f * 1229.0f / 4096.0f + 70.0f - zero_my; // apply calibration offsets in mG that correspond to your environment and magnetometer
    mz = m3 * 10.0f * 1229.0f / 4096.0f + 270.0f - zero_mz;
#endif
}

void Accelerometer::zeroAll(zeroMode action)
{
    zeroAccel(action);
    zeroGyro(action);

#if INCLUDE_MAG_READINGS
    zeroMag(action);
#endif
}

void Accelerometer::zeroAccel(zeroMode action)
{
    if (action == ACCEL_RESET_ZEROS)
    {
        zero_ax = 0;
        zero_ay = 0;
        zero_az = 0;
    }
    else
    {
        zero_ax = ax + zero_ax;
        zero_ay = ay + zero_ay;
        zero_az = az + zero_az;
    }
}

void Accelerometer::zeroGyro(zeroMode action)
{
    if (action == ACCEL_RESET_ZEROS)
    {
        zero_gx = 0;
        zero_gy = 0;
        zero_gz = 0;
    }
    else
    {
        zero_gx = gx + zero_gx;
        zero_gy = gy + zero_gy;
        zero_gz = gz + zero_gz;
    }
}

#if INCLUDE_MAG_READINGS
void Accelerometer::zeroMag(zeroMode action)
{
    if (action == ACCEL_RESET_ZEROS)
    {
        zero_mx = 0;
        zero_my = 0;
        zero_mz = 0;
    }
    else
    {
        zero_mx = mx + zero_mx;
        zero_my = my + zero_my;
        zero_mz = mz + zero_mz;
    }
}
#endif

void Accelerometer::enable()
{
    digitalWrite(interruptPin, LOW);
}
void Accelerometer::disable()
{
    digitalWrite(interruptPin, HIGH);
}

void Accelerometer::printHeader()
{
    dualSerial.println("ax\tay\taz\tgx\tgy\tgz");
}

void Accelerometer::printData()
{
    printAccel();
    printGyro(true);
}

void Accelerometer::printAccel(bool printEndLine)
{
    dualSerial.print(ax);
    dualSerial.print("\t");
    dualSerial.print(ay);
    dualSerial.print("\t");
    dualSerial.print(az);
    dualSerial.print("\t");

    if (printEndLine)
        dualSerial.print("\n");
}
void Accelerometer::printGyro(bool printEndLine)
{
    dualSerial.print(gx);
    dualSerial.print("\t");
    dualSerial.print(gy);
    dualSerial.print("\t");
    dualSerial.print(gz);
    dualSerial.print("\t");

    if (printEndLine)
        dualSerial.print("\n");
}

double Accelerometer::getRotationRate()
{
    // Near max speed
    if (abs(gz) > (MAX_GYRO_READING - MAX_GYRO_READING_TRESHOLD))
    {
        float averageAX = 0;
        for (uint16_t i = 0; i < pastAX.numElements(); i++)
        {
            averageAX += pastAX.getFIFO(i);
        }
        averageAX /= pastAX.numElements() * 1.0;
        return sqrt(averageAX * AX_TO_A_ON_R) / RADS_TO_REVS;
    }

    return gz / 360.0;
}