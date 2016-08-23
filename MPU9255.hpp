#ifndef MPU9255_HPP
#define MPU9255_HPP

#include <functional>

// Utility class used as a convenient way to store values that
// exits in three axes
template<typename T>
struct Vector3
{
    T x;        // Left is positive, right is negative
    T y;        // Back is positive, front is negative 
    T z;        // Up is positive  , down is negative
    
    Vector3(T _x, T _y, T _z):
            x(_x),
            y(_y),
            z(_z)
    {}

    Vector3():
            x(0),
            y(0),
            z(0)
    {}

    Vector3 operator+(Vector3 rhs)
    {
        Vector3 v;
        v.x = x + rhs.x;
        v.y = y + rhs.y;
        v.z = z + rhs.z;

        return v;
    }

    void operator+=(Vector3<T> rhs)
    {
        *this = *this + rhs;
    }

    Vector3 operator-(Vector3 rhs)
    {
        Vector3 v;
        v.x = x - rhs.x;
        v.y = y - rhs.y;
        v.z = z - rhs.z;

        return v;
    }

    void operator-=(Vector3 rhs)
    {
        *this = *this - rhs;
    }

    Vector3 operator*(T rhs)
    {
        Vector3 v;
        v.x = x * rhs;
        v.y = y * rhs;
        v.z = z * rhs;

        return v;
    }

    Vector3 operator/(T rhs)
    {
        Vector3 v;
        v.x = x / rhs;
        v.y = y / rhs;
        v.z = z / rhs;

        return v;
    }

    void operator/=(T rhs)
    {
        *this = *this / rhs;
    }

    template<typename H>
    void operator=(Vector3<H> rhs)
    {
        this->x = static_cast<T>(rhs.x);
        this->y = static_cast<T>(rhs.y);
        this->z = static_cast<T>(rhs.z);
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