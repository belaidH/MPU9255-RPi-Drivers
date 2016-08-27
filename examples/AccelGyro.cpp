#include <iostream>
#include <MPU9255.hpp>

int main() 
{
    MPU9255 imu;
    std::cout << "Gyroscope" << std::endl;
    for(int i = 0; i < 10; i++)
    {
        auto rates = imu.getRotationRates();
        std::cout << "X: " << rates.x << " Y: " << rates.y << " Z: " << rates.z << std::endl;
    }

    std::cout << "Accelerometer" << std::endl;
    for(int i = 0; i < 10; i++)
    {
        auto forces = imu.getAccelerations();
        std::cout << "X: " << forces.x << " Y: " << forces.y << " Z: " << forces.z << std::endl;
    }

    return 0;
}