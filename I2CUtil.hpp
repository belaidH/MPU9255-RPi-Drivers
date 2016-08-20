#ifndef I2CUTIL_HPP
#define I2CUTIL_HPP

#include <wiringPiI2C.h>

namespace I2CUtil
{
    /**
    * @brief Read a 16 bit value from two adjacent 8 bit registers over I2C
    * @param device The Linux filehandle of the I2C device
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
}

#endif