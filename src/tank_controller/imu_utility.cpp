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


#include "tank_controller/imu_utility.h"
using namespace imu;


// ##### Public methods #####


AdafruitBNO055::AdafruitBNO055(int32_t sensor_id, uint8_t address) {
    sensor_id_ = sensor_id;
    i2c_handle_ = wiringPiI2CSetup(address);
}


bool AdafruitBNO055::begin(OperatingMode mode) {
    // Check if wiring pi has successfully initialized
    if (i2c_handle_ == -1) {
        std::cerr << "ERROR: WiringPi setup has failed!" << std::endl;
        return false;
    }

    // Check whether connection was established with correct device
    uint8_t id = readByte_(Registers::CHIP_ID);
    if (id != IMU_ID) {
        // If not give device 1s to boot
        delay(1000);
        id = readByte_(Registers::CHIP_ID);
        // Check again
        if (id != IMU_ID) {
            std::cerr << "ERROR: Couldn't connect with right device!" << std::endl;
            return false;
        }
    }

    // Switch to config mode
    setMode(OperatingMode::CONFIG);

    // Reset
    writeByte_(Registers::SYS_TRIGGER, 0x20);

    // Wait for device to restart
    delay(30);
    while (readByte_(Registers::CHIP_ID) != IMU_ID) {
        delay(10);
    }
    delay(50);

    // Return to normal powermode
    writeByte_(Registers::PWR_MODE, Powermodes::NORMAL);
    delay(10);
    writeByte_(Registers::PAGE_ID, 0);
    writeByte_(Registers::SYS_TRIGGER, 0x0);
    delay(10);

    // Set the requested operating mode
    setMode(mode);
    delay(20);

    return true;
}


void AdafruitBNO055::setMode(OperatingMode mode) {
    mode_ = mode;
    writeByte_(Registers::OPR_MODE, mode);
    delay(30);
}


void AdafruitBNO055::getMode(OperatingMode &mode) {
    mode = (OperatingMode) readByte_(Registers::OPR_MODE);
}


void AdafruitBNO055::setAxisRemap(RemapConfigs remapcode) {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    // Remap
    writeByte_(Registers::AXIS_MAP_CONFIG, remapcode);
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);
}


void AdafruitBNO055::setAxisSign(RemapSigns remapsign) {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    // Remap
    writeByte_(Registers::AXIS_MAP_SIGN, remapsign);
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);
}


void AdafruitBNO055::getRevInfo(Revisions &rev){
    // Get accelerometer revisions
    rev.accelerometer= readByte_(Registers::ACCEL_REV_ID);

    // Get magnetometer revisions
    rev.magnetometer = readByte_(Registers::MAG_REV_ID);

    // Get gyroscope revisions
    rev.gyroscope = readByte_(Registers::GYRO_REV_ID);

    // Get bootloader revisions
    rev.bootloader = readByte_(Registers::BL_REV_ID);

    // Get SW revisions (bit operations are required here)
    uint8_t lsb, msb;
    lsb = readByte_(Registers::SW_REV_ID_LSB);
    msb = readByte_(Registers::SW_REV_ID_MSB);
    rev.sw = (((uint16_t) msb) << 8) | ((uint16_t) lsb);
}


void AdafruitBNO055::setExtCrystalUse(bool usextal) {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    // Write appropriate value to register
    writeByte_(PAGE_ID, 0);
    if (usextal) {
        writeByte_(SYS_TRIGGER, 0x80);
    } else {
        writeByte_(SYS_TRIGGER, 0x00);
    }
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);
}


void AdafruitBNO055::getSystemStatus(uint8_t &system_status, uint8_t &self_test_result, uint8_t &system_error) {
    writeByte_(Registers::PAGE_ID, 0);

    // Read data from appropriate registers
    system_status = readByte_(Registers::SYS_STAT);
    self_test_result = readByte_(Registers::SELFTEST_RESULT);
    system_error = readByte_(Registers::SYS_ERR);
    delay(200);
}


void AdafruitBNO055::getCalibration(uint8_t &system_status, uint8_t &gyro, uint8_t &accel, uint8_t &mag) {
    // Read data, then separate informations with bit-wise operations
    uint8_t data = readByte_(Registers::CALIB_STAT);
    system_status = (data >> 6) & 0x03;
    gyro = (data >> 4) & 0x03;
    accel = (data >> 2) & 0x03;
    mag = data & 0x03;
}


void AdafruitBNO055::getVector(float vector[3], VectorType vector_type) {
    // Create buffers
    uint8_t buffer[6] = {0, 0, 0, 0, 0, 0};
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    // Read vector data and split it into buffers
    readLen_((Registers) vector_type, buffer, 6);
    x = ((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8);
    y = ((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8);
    z = ((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8);

    // Covert value to an appropriate range and write it to output array
    switch (vector_type) {
        case VectorType::MAGNETOMETER:
            // 1 uT = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorType::GYROSCOPE:
            // 1 dps = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorType::EULER:
            // 1 degree = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorType::ACCELEROMETER:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
        case VectorType::LINEARACCEL:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
        case VectorType::GRAVITY:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
  }
}


void AdafruitBNO055::getQuaternion(float vector[4]) {
    // Create buffers
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int16_t w = 0;

    // Read and split data
    readLen_(Registers::QUATERNION_DATA_W_LSB, buffer, 8);
    w = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
    x = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[2]);
    y = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[4]);
    z = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[6]);

    // Assign data to quaternion
    const float scale = (1.0f / (1 << 14));
    vector[0] = scale * x;
    vector[1] = scale * y;
    vector[2] = scale * z;
    vector[3] = scale * w;
}


void AdafruitBNO055::getTemperature(int8_t &temp) {
    temp = (int8_t) (readByte_(Registers::TEMP));
}


bool AdafruitBNO055::getSensorOffsets(uint8_t calib_data[IMU_OFFSET_REGISTERS]) {
    if (isFullyCalibrated()) {
        // Backup previous operating mode and temporarily change to config mode
        OperatingMode prev_mode = mode_;
        setMode(OperatingMode::CONFIG);
        delay(25);

        readLen_(Registers::ACCEL_OFFSET_X_LSB, calib_data, IMU_OFFSET_REGISTERS);

        // Return to previous operating mode
        setMode(prev_mode);
    return true;
    }
    return false;
}



bool AdafruitBNO055::getSensorOffsets(Offsets &offset_type) {
    if (isFullyCalibrated()) {
        // Backup previous operating mode and temporarily change to config mode
        OperatingMode prev_mode = mode_;
        setMode(OperatingMode::CONFIG);
        delay(25);

        /* Accel offset range depends on the G-range:
        +/-2 g  = +/- 2000 mg
        +/-4 g  = +/- 4000 mg
        +/-8 g  = +/- 8000 mg
        +/-16 g = +/- 16000 mg */
        offset_type.accel_x = (readByte_(Registers::ACCEL_OFFSET_X_MSB) << 8) | (readByte_(Registers::ACCEL_OFFSET_X_LSB));
        offset_type.accel_y = (readByte_(Registers::ACCEL_OFFSET_Y_MSB) << 8) | (readByte_(Registers::ACCEL_OFFSET_Y_LSB));
        offset_type.accel_z = (readByte_(Registers::ACCEL_OFFSET_Z_MSB) << 8) | (readByte_(Registers::ACCEL_OFFSET_Z_LSB));

        /* Magnetometer offset range = +/- 6400 LSB where 1 uT = 16 LSB */
        offset_type.mag_x = (readByte_(Registers::MAG_OFFSET_X_MSB) << 8) | (readByte_(Registers::MAG_OFFSET_X_LSB));
        offset_type.mag_y = (readByte_(Registers::MAG_OFFSET_Y_MSB) << 8) | (readByte_(Registers::MAG_OFFSET_Y_LSB));
        offset_type.mag_z = (readByte_(Registers::MAG_OFFSET_Z_MSB) << 8) | (readByte_(Registers::MAG_OFFSET_Z_LSB));

        /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB */
        offset_type.gyro_x = (readByte_(Registers::GYRO_OFFSET_X_MSB) << 8) | (readByte_(Registers::GYRO_OFFSET_X_LSB));
        offset_type.gyro_y = (readByte_(Registers::GYRO_OFFSET_Y_MSB) << 8) | (readByte_(Registers::GYRO_OFFSET_Y_LSB));
        offset_type.gyro_z = (readByte_(Registers::GYRO_OFFSET_Z_MSB) << 8) | (readByte_(Registers::GYRO_OFFSET_Z_LSB));

        /* Accelerometer radius = +/- 1000 LSB */
        offset_type.accel_radius = (readByte_(Registers::ACCEL_RADIUS_MSB) << 8) | (readByte_(Registers::ACCEL_RADIUS_LSB));

        /* Magnetometer radius = +/- 960 LSB */
        offset_type.mag_radius = (readByte_(Registers::MAG_RADIUS_MSB) << 8) | (readByte_(Registers::MAG_RADIUS_LSB));

        // Return to previous operating mode
        setMode(prev_mode);
        
        return true;
    }
    return false;
}


void AdafruitBNO055::setSensorOffsets(const uint8_t calib_data[IMU_OFFSET_REGISTERS]) {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */
    writeByte_(Registers::ACCEL_OFFSET_X_LSB, calib_data[0]);
    writeByte_(Registers::ACCEL_OFFSET_X_MSB, calib_data[1]);
    writeByte_(Registers::ACCEL_OFFSET_Y_LSB, calib_data[2]);
    writeByte_(Registers::ACCEL_OFFSET_Y_MSB, calib_data[3]);
    writeByte_(Registers::ACCEL_OFFSET_Z_LSB, calib_data[4]);
    writeByte_(Registers::ACCEL_OFFSET_Z_MSB, calib_data[5]);

    writeByte_(Registers::MAG_OFFSET_X_LSB, calib_data[6]);
    writeByte_(Registers::MAG_OFFSET_X_MSB, calib_data[7]);
    writeByte_(Registers::MAG_OFFSET_Y_LSB, calib_data[8]);
    writeByte_(Registers::MAG_OFFSET_Y_MSB, calib_data[9]);
    writeByte_(Registers::MAG_OFFSET_Z_LSB, calib_data[10]);
    writeByte_(Registers::MAG_OFFSET_Z_MSB, calib_data[11]);

    writeByte_(Registers::GYRO_OFFSET_X_LSB, calib_data[12]);
    writeByte_(Registers::GYRO_OFFSET_X_MSB, calib_data[13]);
    writeByte_(Registers::GYRO_OFFSET_Y_LSB, calib_data[14]);
    writeByte_(Registers::GYRO_OFFSET_Y_MSB, calib_data[15]);
    writeByte_(Registers::GYRO_OFFSET_Z_LSB, calib_data[16]);
    writeByte_(Registers::GYRO_OFFSET_Z_MSB, calib_data[17]);

    writeByte_(Registers::ACCEL_RADIUS_LSB, calib_data[18]);
    writeByte_(Registers::ACCEL_RADIUS_MSB, calib_data[19]);

    writeByte_(Registers::MAG_RADIUS_LSB, calib_data[20]);
    writeByte_(Registers::MAG_RADIUS_MSB, calib_data[21]);

    // Return to previous operating mode
    setMode(prev_mode);
}


void AdafruitBNO055::setSensorOffsets(const Offsets offset_type) {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */
    writeByte_(Registers::ACCEL_OFFSET_X_LSB, (offset_type.accel_x) & 0x0FF);
    writeByte_(Registers::ACCEL_OFFSET_X_MSB, (offset_type.accel_x >> 8) & 0x0FF);
    writeByte_(Registers::ACCEL_OFFSET_Y_LSB, (offset_type.accel_y) & 0x0FF);
    writeByte_(Registers::ACCEL_OFFSET_Y_MSB, (offset_type.accel_y >> 8) & 0x0FF);
    writeByte_(Registers::ACCEL_OFFSET_Z_LSB, (offset_type.accel_z) & 0x0FF);
    writeByte_(Registers::ACCEL_OFFSET_Z_MSB, (offset_type.accel_z >> 8) & 0x0FF);

    writeByte_(Registers::MAG_OFFSET_X_LSB, (offset_type.mag_x) & 0x0FF);
    writeByte_(Registers::MAG_OFFSET_X_MSB, (offset_type.mag_x >> 8) & 0x0FF);
    writeByte_(Registers::MAG_OFFSET_Y_LSB, (offset_type.mag_y) & 0x0FF);
    writeByte_(Registers::MAG_OFFSET_Y_MSB, (offset_type.mag_y >> 8) & 0x0FF);
    writeByte_(Registers::MAG_OFFSET_Z_LSB, (offset_type.mag_z) & 0x0FF);
    writeByte_(Registers::MAG_OFFSET_Z_MSB, (offset_type.mag_z >> 8) & 0x0FF);

    writeByte_(Registers::GYRO_OFFSET_X_LSB, (offset_type.gyro_x) & 0x0FF);
    writeByte_(Registers::GYRO_OFFSET_X_MSB, (offset_type.gyro_x >> 8) & 0x0FF);
    writeByte_(Registers::GYRO_OFFSET_Y_LSB, (offset_type.gyro_y) & 0x0FF);
    writeByte_(Registers::GYRO_OFFSET_Y_MSB, (offset_type.gyro_y >> 8) & 0x0FF);
    writeByte_(Registers::GYRO_OFFSET_Z_LSB, (offset_type.gyro_z) & 0x0FF);
    writeByte_(Registers::GYRO_OFFSET_Z_MSB, (offset_type.gyro_z >> 8) & 0x0FF);

    writeByte_(Registers::ACCEL_RADIUS_LSB, (offset_type.accel_radius) & 0x0FF);
    writeByte_(Registers::ACCEL_RADIUS_MSB, (offset_type.accel_radius >> 8) & 0x0FF);

    writeByte_(Registers::MAG_RADIUS_LSB, (offset_type.mag_radius) & 0x0FF);
    writeByte_(Registers::MAG_RADIUS_MSB, (offset_type.mag_radius >> 8) & 0x0FF);

    // Return to previous operating mode
    setMode(prev_mode);
}


 bool AdafruitBNO055::isFullyCalibrated() {
    // Read calibration state
    uint8_t system_status = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
    getCalibration(system_status, gyro, accel, mag);

    // Check calibration state given current operating mode
    switch (mode_) {
        case OperatingMode::ACCONLY:
            return (accel == 3);
        case OperatingMode::MAGONLY:
            return (mag == 3);
        case OperatingMode::GYRONLY:
        case OperatingMode::M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OperatingMode::ACCMAG:
        case OperatingMode::COMPASS:
            return (accel == 3 && mag == 3);
        case OperatingMode::ACCGYRO:
        case OperatingMode::IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OperatingMode::MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system_status == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
 }


void AdafruitBNO055::enterSuspendMode() {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    writeByte_(Registers::PWR_MODE, 0x02);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);
}


void AdafruitBNO055::enterNormalMode() {
    // Backup previous operating mode and temporarily change to config mode
    OperatingMode prev_mode = mode_;
    setMode(OperatingMode::CONFIG);
    delay(25);

    writeByte_(Registers::PWR_MODE, 0x00);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);
}


// ##### Private methods #####


uint8_t AdafruitBNO055::readByte_(Registers reg) {
    return (uint8_t) wiringPiI2CReadReg8(i2c_handle_, reg);   // This could probably work, right?
}


bool AdafruitBNO055::readLen_(Registers reg, uint8_t buffer[], uint8_t len) {
    try{
        for (uint8_t i=0; i < len; i++) {
            buffer[i] = (uint8_t) wiringPiI2CReadReg8(i2c_handle_, reg + i);
        }
    } catch (...) {
        return false;
    }
    return true;
}


bool AdafruitBNO055::writeByte_(Registers reg, uint8_t value) {
    int result =  wiringPiI2CWriteReg8(i2c_handle_, reg, value);
    return (result != -1 ? true : false);   // Yes, this could be simplified but this way it's more clear what this is supposed to do
}