#include <iostream>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <array>
#include <chrono>
#include <thread>
#include <functional>

#include <wiringPiI2C.h>

template<typename T>
struct Vector3
{
    T x;        // Left is positive, right is negative
    T y;        // Back is positive, front is negative 
    T z;        // Up is positive  , down is negative
};

/**
 * @brief Read a 16 bit value from two adjacent 8 bit registers over I2C
 * @param device The linux filehandle of the I2C device
 * @param reg The first register to be read from
 * 
 * @return The 16 bit integer
 */
int16_t readWord(int device, int reg)
{
    int8_t highByte = wiringPiI2CReadReg8(device, reg);
    int8_t lowByte = wiringPiI2CReadReg8(device, reg + 1);

    return highByte << 8 | lowByte;
}

Vector3<int16_t> calcSensorOffset(int device, std::function<Vector3<int16_t>(int)> sensorAccessFunc, int trials = 20)
{
    assert(trials > 0);

    Vector3<int16_t> offsets;
    for(int i = 0; i < trials; i++)
    {
        auto data = sensorAccessFunc(device);
        offsets.x += data.x;
        offsets.y += data.y;
        offsets.z += data.z;
    }

    offsets.x /= trials;
    offsets.y /= trials;
    offsets.z /= trials;

    return offsets;    
}

/**
 * @brief Read the accelerometer
 * @param device The linux filehandle of the I2C device
 * 
 * @return The raw values from the accelerometer for each axis
 */
Vector3<int16_t> getRawAccelData(int device)
{
    // Measure the acceleration in each axis
    Vector3<int16_t> accelXYZ;
    accelXYZ.x = readWord(device, 59);
    accelXYZ.y = readWord(device, 61);
    accelXYZ.z = readWord(device, 63);

    return accelXYZ;
}

/**
 * @brief Read the gyroscope
 * @param device The linux filehandle of the I2C device
 * 
 * @return The raw values from the gyroscope for each axis
 */
Vector3<int16_t> getRawGyroData(int device)
{
    // Measure the rotation in each axis
    Vector3<int16_t> gyroXYZ;
    gyroXYZ.x = readWord(device, 67);
    gyroXYZ.y = readWord(device, 69);
    gyroXYZ.z = readWord(device, 71);

    return gyroXYZ;
}

// Accelerometer sensitivities
const double AccelSenFactor2G  = 16384.0;
const double AccelSenFactor4G  = 8192.0;
const double AccelSenFactor8G  = 4096.0;
const double AccelSenFactor16G = 2048.0;

// Gyroscope sensitivities
const double GyroSenFactor250  = 131.0;
const double GyroSenFactor500  = 65.5;
const double GyroSenFactor1000 = 32.8;
const double GyroSenFactor2000 = 16.4;

int main() 
{
    // Setup I2C device
    int device{wiringPiI2CSetup(0x68)};
    if(device == -1) 
    {
        std::cout << "Failed to setup device!" << std::endl;
        return -1;
    }

    // Wake-up device
    wiringPiI2CWriteReg8(device, 0x6b, 0);

    // Set accelerometer sensitivity to +/- 16g
    wiringPiI2CWriteReg8(device, 28, 12);
    // wiringPiI2CWriteReg8(device, 30, 11);

    // Calculuate the sensor offsets assuming there is no
    // acceleration or rotation
    Vector3<int16_t> accelOffsets = calcSensorOffset(device, getRawAccelData); 
    Vector3<int16_t> gyroOffsets = calcSensorOffset(device, getRawGyroData);

    // Display some measurements from the gyroscope
    std::cout << "Gyroscope" << std::endl;
    for(int i = 0; i < 10; i++)
    {
        // The x, y, and z axes are offseted to zero degrees/sec
        Vector3<int16_t> gyroData = getRawGyroData(device);
        std::cout << "X: " << (gyroData.x - gyroOffsets.x) / GyroSenFactor250 << ", "
                  << "Y: " << (gyroData.y - gyroOffsets.y) / GyroSenFactor250 << ", "
                  << "Z: " << (gyroData.z - gyroOffsets.z) / GyroSenFactor250 << std::endl;
    }

    // Display some measurements from the accelerometer
    std::cout << "Accelerometer" << std::endl;
    for(int i = 0; i < 10; i++)
    {
        // The x and y axes are offseted to zero g's
        // The z axis (vertical axis) is offseted to 1g because there is always gravity
        Vector3<int16_t> accelData = getRawAccelData(device);
        std::cout << "X: " << ( accelData.x - accelOffsets.x ) / AccelSenFactor16G << ", "
                  << "Y: " << ( accelData.y - accelOffsets.y ) / AccelSenFactor16G << ", "
                  << "Z: " << ( accelData.z) / AccelSenFactor16G << std::endl;
    }

    return 0;
}