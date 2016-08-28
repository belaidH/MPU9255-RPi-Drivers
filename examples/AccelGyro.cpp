#include <iostream>
#include <MPU9255.hpp>

int main() 
{
    MPU9255 imu;

    // Get the rotational rates from the gyroscope in units of degrees/second
    std::cout << "Gyroscope" << std::endl;
    auto rates = imu.getRotationRates();
    std::cout << "X: " << rates.x << " Y: " << rates.y << " Z: " << rates.z << std::endl;

    // Get the forces measured from the accelerometer in units of g's (1g of gravity is nominal Earth gravity)
    std::cout << "Accelerometer" << std::endl;
    auto forces = imu.getAccelerations();
    std::cout << "X: " << forces.x << " Y: " << forces.y << " Z: " << forces.z << std::endl;

    return 0;
}