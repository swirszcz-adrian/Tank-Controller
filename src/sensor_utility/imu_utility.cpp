/**
 * @file imu_utility.cpp
 * @author Adrian Swirszcz (swirszczadrian@o2.pl)
 * @brief Port of Adafruit's BNO005 library: https://github.com/adafruit/Adafruit_BNO055
 * 
 * @details The "imu_utility" library (that is constents of "imu_utility.h" and "imu_utility.cpp" files)
 * is a port of Adafruit's BNO055 library for Raspberry PI
 * Link to the original code: https://github.com/adafruit/Adafruit_BNO055
 * 
 * @copyright Original code license:
 *    Copyright (c) 2018 Adafruit Industries
 *    
 *    Permission is hereby granted, free of charge, to any person obtaining a copy
 *    of this software and associated documentation files (the "Software"), to deal
 *    in the Software without restriction, including without limitation the rights
 *    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the Software is
 *    furnished to do so, subject to the following conditions:
 *    
 *    The above copyright notice and this permission notice shall be included in all
 *    copies or substantial portions of the Software.
 *    
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *    SOFTWARE.
 */


#include "sensor_utility/imu_utility.h"


imuAdafruitBNO055::imuAdafruitBNO055(int32_t sensor_id = -1, uint8_t address = IMU_ADRESS) {
    sensor_id_ = sensor_id;
    i2c_handle_ = wiringPiI2CSetup(address);
}


uint8_t imuAdafruitBNO055::readByte_(registers reg) {
    return (uint8_t) wiringPiI2CReadReg8(i2c_handle_, reg);   // This could probably work, right?
}


bool imuAdafruitBNO055::readLen_(registers reg, uint8_t (&buffer)[], uint8_t len) {
    try{
        for (uint8_t i=0; i < len; i++) {
            buffer[i] = (uint8_t) wiringPiI2CReadReg8(i2c_handle_, reg + i);
        }
    } catch (...) {
        return false;
    }
    return true;
}


bool imuAdafruitBNO055::writeByte_(registers reg, uint8_t value) {
    return (bool) wiringPiI2CWriteReg8(i2c_handle_, reg, value);
}