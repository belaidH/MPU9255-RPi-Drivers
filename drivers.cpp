#include <iostream>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <array>

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

int16_t calcAccelOffset(int device, int accReg, int trials = 20)
{
    assert(trials > 0);

    int total = 0;
    for(int i = 0; i < trials; i++)
        total += readWord(device, accReg);

    int offset = total /= trials;
    return offset;    
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

// When range is +/- 2g
const double AccelSenFactor = 16384.0;

int main() 
{
    // Setup I2C device
    int device = wiringPiI2CSetup(0x68);
    if(device == -1) 
    {
        std::cout << "Failed to setup device!" << std::endl;
        return -1;
    }

    // Wake-up device
    wiringPiI2CWriteReg8(device, 0x6b, 0);
     
    // Calculuate the offset for the accelerometer
    // assuming it is level with the horizon
    Vector3<int16_t> accelOffsets; 
    accelOffsets.x = calcAccelOffset(device, 59);
    accelOffsets.y = calcAccelOffset(device, 61);
    accelOffsets.z = calcAccelOffset(device, 63);

    std::cout << accelOffsets.z / AccelSenFactor << std::endl;

    // Display some measurements from the IMU 
    for(int i = 0; i < 10; i++){

        // The x and y axis are offseted to zero g's
        // The z axis (vertical axis) is offseted to 1g because there is always gravity
        Vector3<int16_t> accelData = getRawAccelData(device);
        std::cout << "X: " << ( accelData.x - accelOffsets.x ) / AccelSenFactor << ", "
                  << "Y: " << ( accelData.y - accelOffsets.y ) / AccelSenFactor << ", "
                  << "Z: " << ( accelData.z - accelOffsets.z + 1) / AccelSenFactor << std::endl;
    }

    std::cout << std::endl;
    return 0;
}