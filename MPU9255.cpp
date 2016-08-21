#include <MPU9255.hpp>
#include <wiringPiI2C.h>
#include <cassert>
#include <vector>
#include <iostream>

#include <I2CUtil.hpp>

// The accelerometers sensitivities for all four
// measurement ranges in units of LSB/g
namespace AccelSenFactors
{
    const double g2 = 16384.0;
    const double g4 = 8192.0;
    const double g8 = 4096.0;
    const double g16 = 2048.0;
}

// The gyroscope sensitivities for all four
// measurement ranges in units of LSB/(degree/s)
namespace GyroSenFactor
{
    const double ds250 = 131.0;
    const double ds500 = 65.5;
    const double ds1000 = 32.8;
    const double ds2000 = 16.4;        
}

/**
* @brief Calculate the average sensor value over trials for use as a sensor offset
* @param sensorAccessFunc The function that returns the sensor values
* @param trials The number of samples to average
*/
Vector3<int16_t> calcSensorOffset(std::function<Vector3<int16_t>()> sensorAccessFunc, int trials = 5)
{
    assert(trials > 0);

    Vector3<int16_t> offsets;
    for(int i = 0; i < trials; i++)
        offsets += sensorAccessFunc();

    offsets /= trials;
    return offsets;    
}

MPU9255::MPU9255()
{
    // Setup IMU
    mDeviceHandle = wiringPiI2CSetup(0x68);
    if(mDeviceHandle == -1)
        std::cout << "Failed to setup device!" << std::endl;

    // Wake up IMU
    wiringPiI2CWriteReg8(mDeviceHandle, 0x6b, 0);

    // Calculate the accelerometer and gyroscope 
    mGyroOffsets = calcSensorOffset(std::bind(&MPU9255::getRawGyroData, this));
    mAccelOffsets = calcSensorOffset(std::bind(&MPU9255::getRawAccelData, this));

    // My default the accelerometer and gyroscope are set 
    // to measure in the +/- 2g and +/- 250 deg/s range
    mAccelSenFactor = AccelSenFactors::g2;
    mGyroSenFactor = GyroSenFactor::ds250;

    auto data = getRawGyroData();
}

Vector3<double> MPU9255::getRotationRates() const
{
    auto rawRates = getRawGyroData();
    rawRates -= mGyroOffsets;

    Vector3<double> rotationRates;
    rotationRates = rawRates;
    rotationRates /= mGyroSenFactor;

    return rotationRates;
}

Vector3<double> MPU9255::getAccelerations() const
{
    auto rawRates = getRawAccelData();
    rawRates -= mAccelOffsets;
    
    Vector3<double> accelerations;
    accelerations = rawRates;
    accelerations /= mAccelSenFactor;
    accelerations.z += 1.0;     // Account for the constant 1g of gravity
 
    return accelerations;
}

/**
 * @brief Read the accelerometer
 * 
 * @return The raw values from the accelerometer for each axis
 */
Vector3<int16_t> MPU9255::getRawAccelData() const
{
    // Measure the acceleration in each axis
    Vector3<int16_t> accelXYZ;
    accelXYZ.x = I2CUtil::readWord(mDeviceHandle, 59);
    accelXYZ.y = I2CUtil::readWord(mDeviceHandle, 61);
    accelXYZ.z = I2CUtil::readWord(mDeviceHandle, 63);

    return accelXYZ;
}

/**
 * @brief Read the gyroscope
 * 
 * @return The raw values from the gyroscope for each axis
 */
Vector3<int16_t> MPU9255::getRawGyroData() const
{
    // Measure the rotation in each axis
    Vector3<int16_t> gyroXYZ;
    gyroXYZ.x = I2CUtil::readWord(mDeviceHandle, 67);
    gyroXYZ.y = I2CUtil::readWord(mDeviceHandle, 69);
    gyroXYZ.z = I2CUtil::readWord(mDeviceHandle, 71);

    return gyroXYZ;
}
