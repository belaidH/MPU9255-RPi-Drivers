#ifndef MPU9255_HPP
#define MPU9255_HPP

#include <functional>

template<typename T>
struct Vector3
{
    T x;        // Left is positive, right is negative
    T y;        // Back is positive, front is negative 
    T z;        // Up is positive  , down is negative
    
    Vector3 operator+(Vector3& rhs)
    {
        Vector3 v;
        v.x = x + rhs.x;
        v.y = y + rhs.y;
        v.z = z + rhs.z;

        return v;
    }

    void operator+=(Vector3& rhs)
    {
        *this = *this + rhs;
    }

    Vector3 operator-(Vector3& rhs)
    {
        Vector3 v;
        v.x = x - rhs.x;
        v.y = y - rhs.y;
        v.z = z - rhs.z;

        return v;
    }

    void operator-=(Vector3& rhs)
    {
        *this = *this - rhs;
    }

    Vector3 operator*(const T& rhs)
    {
        Vector3 v;
        v.x = x * rhs;
        v.y = y * rhs;
        v.z = z * rhs;

        return v;
    }

    Vector3 operator/(const T& rhs)
    {
        Vector3 v;
        v.x = x / rhs;
        v.y = y / rhs;
        v.z = z / rhs;

        return v;
    }

    void operator/=(const T& rhs)
    {
        *this = *this / rhs;
    }
};

class MPU9255
{
public:
    MPU9255();
    Vector3<double> getRotationRates() const;
    Vector3<double> getAccelerations() const;
protected:
    // The Linux filehandle of the IMU
    int mDeviceHandle;

    Vector3<int16_t> getRawGyroData() const;
    Vector3<int16_t> getRawAccelData() const;

    // The accelerometer and gyroscope often
    // report motion when there is none and thus
    // must be offset
    Vector3<int16_t> mGyroOffsets;
    Vector3<int16_t> mAccelOffsets;

    double mAccelSenFactor;  // LSB/g
    double mGyroSenFactor;   // LSB/(degree/s)

};

#endif