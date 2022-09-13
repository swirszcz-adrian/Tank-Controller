/**
 * @file imu_utility.h
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
#ifndef IMU_UTILITY_H
#define IMU_UTILITY_H


#include <wiringPiI2C.h>
#include <stdint.h>
#include <vector>


// Define basic constants
#define IMU_ADRESS (0x28)
#define IMU_ID (0xA0)


/**
 * @brief Struct holding offset values for Adafruit BNO055 sensor
 */
struct imuOffsets {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_radius;
    
    int16_t mag_radius;
};


/**
 * @brief Enumeration for storing Adafruit BNO005 sensor's operating modes
 */
enum imuOperatingMode {
    CONFIG = 0X00,
    ACCONLY = 0X01,
    MAGONLY = 0X02,
    GYRONLY = 0X03,
    ACCMAG = 0X04,
    ACCGYRO = 0X05,
    MAGGYRO = 0X06,
    AMG = 0X07,
    IMUPLUS = 0X08,
    COMPASS = 0X09,
    M4G = 0X0A,
    NDOF_FMC_OFF = 0X0B,
    NDOF = 0X0C
};


/**
 * @brief Class containing all the necessary components to interface with Adafruit BNO005 IMU sensor via I2C protocol
 */
class imuAdafruitBNO055 {
public:
    // DATA SECTION
    
    /**
     * @brief List of all registers that will be used when communicating with Adafruit BNO055 sensor
     */
    enum registers {
        // Basic registers
        CHIP_ID = 0x00,
        ACCEL_REV_ID = 0x01,
        MAG_REV_ID = 0x02,
        GYRO_REV_ID = 0x03,
        SW_REV_ID_LSB = 0x04,
        SW_REV_ID_MSB = 0x05,
        BL_REV_ID = 0X06,
        PAGE_ID = 0X07,

        // Acceleration data register
        ACCEL_DATA_X_LSB = 0X08,
        ACCEL_DATA_X_MSB = 0X09,
        ACCEL_DATA_Y_LSB = 0X0A,
        ACCEL_DATA_Y_MSB = 0X0B,
        ACCEL_DATA_Z_LSB = 0X0C,
        ACCEL_DATA_Z_MSB = 0X0D,

        // Magnetic field data registers
        MAG_DATA_X_LSB = 0X0E,
        MAG_DATA_X_MSB = 0X0F,
        MAG_DATA_Y_LSB = 0X10,
        MAG_DATA_Y_MSB = 0X11,
        MAG_DATA_Z_LSB = 0X12,
        MAG_DATA_Z_MSB = 0X13,

        // Gyro data registers
        GYRO_DATA_X_LSB = 0X14,
        GYRO_DATA_X_MSB = 0X15,
        GYRO_DATA_Y_LSB = 0X16,
        GYRO_DATA_Y_MSB = 0X17,
        GYRO_DATA_Z_LSB = 0X18,
        GYRO_DATA_Z_MSB = 0X19,

        // Euler data registers
        EULER_H_LSB = 0X1A,
        EULER_H_MSB = 0X1B,
        EULER_R_LSB = 0X1C,
        EULER_R_MSB = 0X1D,
        EULER_P_LSB = 0X1E,
        EULER_P_MSB = 0X1F,

        // Quaternion data registers
        QUATERNION_DATA_W_LSB = 0X20,
        QUATERNION_DATA_W_MSB = 0X21,
        QUATERNION_DATA_X_LSB = 0X22,
        QUATERNION_DATA_X_MSB = 0X23,
        QUATERNION_DATA_Y_LSB = 0X24,
        QUATERNION_DATA_Y_MSB = 0X25,
        QUATERNION_DATA_Z_LSB = 0X26,
        QUATERNION_DATA_Z_MSB = 0X27,

        // Linear acceleration data registers
        LINEAR_ACCEL_DATA_X_LSB = 0X28,
        LINEAR_ACCEL_DATA_X_MSB = 0X29,
        LINEAR_ACCEL_DATA_Y_LSB = 0X2A,
        LINEAR_ACCEL_DATA_Y_MSB = 0X2B,
        LINEAR_ACCEL_DATA_Z_LSB = 0X2C,
        LINEAR_ACCEL_DATA_Z_MSB = 0X2D,

        // Gravity data registers
        GRAVITY_DATA_X_LSB = 0X2E,
        GRAVITY_DATA_X_MSB = 0X2F,
        GRAVITY_DATA_Y_LSB = 0X30,
        GRAVITY_DATA_Y_MSB = 0X31,
        GRAVITY_DATA_Z_LSB = 0X32,
        GRAVITY_DATA_Z_MSB = 0X33,

        // Temperature data register
        TEMP = 0X34,

        // Status registers
        CALIB_STAT = 0X35,
        SELFTEST_RESULT = 0X36,
        INTR_STAT = 0X37,
        SYS_CLK_STAT = 0X38,
        SYS_STAT = 0X39,
        SYS_ERR = 0X3A,

        // Unit selection register
        UNIT_SEL = 0X3B,

        // Mode registers
        OPR_MODE = 0X3D,
        PWR_MODE = 0X3E,
        SYS_TRIGGER = 0X3F,
        TEMP_SOURCE = 0X40,

        // Axis remap registers
        AXIS_MAP_CONFIG = 0X41,
        AXIS_MAP_SIGN = 0X42,

        // SIC registers
        SIC_MATRIX_0_LSB = 0X43,
        SIC_MATRIX_0_MSB = 0X44,
        SIC_MATRIX_1_LSB = 0X45,
        SIC_MATRIX_1_MSB = 0X46,
        SIC_MATRIX_2_LSB = 0X47,
        SIC_MATRIX_2_MSB = 0X48,
        SIC_MATRIX_3_LSB = 0X49,
        SIC_MATRIX_3_MSB = 0X4A,
        SIC_MATRIX_4_LSB = 0X4B,
        SIC_MATRIX_4_MSB = 0X4C,
        SIC_MATRIX_5_LSB = 0X4D,
        SIC_MATRIX_5_MSB = 0X4E,
        SIC_MATRIX_6_LSB = 0X4F,
        SIC_MATRIX_6_MSB = 0X50,
        SIC_MATRIX_7_LSB = 0X51,
        SIC_MATRIX_7_MSB = 0X52,
        SIC_MATRIX_8_LSB = 0X53,
        SIC_MATRIX_8_MSB = 0X54,

        // Accelerometer offset registers
        ACCEL_OFFSET_X_LSB = 0X55,
        ACCEL_OFFSET_X_MSB = 0X56,
        ACCEL_OFFSET_Y_LSB = 0X57,
        ACCEL_OFFSET_Y_MSB = 0X58,
        ACCEL_OFFSET_Z_LSB = 0X59,
        ACCEL_OFFSET_Z_MSB = 0X5A,

        // Magnetometer offset registers
        MAG_OFFSET_X_LSB = 0X5B,
        MAG_OFFSET_X_MSB = 0X5C,
        MAG_OFFSET_Y_LSB = 0X5D,
        MAG_OFFSET_Y_MSB = 0X5E,
        MAG_OFFSET_Z_LSB = 0X5F,
        MAG_OFFSET_Z_MSB = 0X60,

        // Gyroscope offset registers
        GYRO_OFFSET_X_LSB = 0X61,
        GYRO_OFFSET_X_MSB = 0X62,
        GYRO_OFFSET_Y_LSB = 0X63,
        GYRO_OFFSET_Y_MSB = 0X64,
        GYRO_OFFSET_Z_LSB = 0X65,
        GYRO_OFFSET_Z_MSB = 0X66,


        // Radius registers
        ACCEL_RADIUS_LSB = 0X67,
        ACCEL_RADIUS_MSB = 0X68,
        MAG_RADIUS_LSB = 0X69,
        MAG_RADIUS_MSB = 0X6A
    };

    /**
     * @brief List of sensor's power settings
     */
    enum powermodes {
        NORMAL = 0X00,
        LOWPOWER = 0X01,
        SUSPEND = 0X02
    };

    /**
     * @brief List of remap settings
     */
    enum remapConfigs {
        P0 = 0x21,
        P1 = 0x24, // default
        P2 = 0x24,
        P3 = 0x21,
        P4 = 0x24,
        P5 = 0x21,
        P6 = 0x21,
        P7 = 0x24
    };

    /**
     * @brief List of remap signs
     */
    enum remapSigns {
        P0 = 0x04,
        P1 = 0x00, // default
        P2 = 0x06,
        P3 = 0x02,
        P4 = 0x03,
        P5 = 0x01,
        P6 = 0x07,
        P7 = 0x05
    };

    /**
     * @brief Struct representing revisions
     */
    struct revisions {
        uint8_t acceleration;
        uint8_t magnetometer;
        uint8_t gyroscope;
        uint16_t sw;
        uint8_t bootloader;
    };

    /**
     * @brief List of useful vector mappings
     */
    enum vectorType {
        ACCELEROMETER = registers::ACCEL_DATA_X_LSB,
        MAGNETOMETER = registers::MAG_DATA_X_LSB,
        GYROSCOPE = registers::GYRO_DATA_X_LSB,
        EULER = registers::EULER_H_LSB,
        LINEARACCEL = registers::LINEAR_ACCEL_DATA_X_LSB,
        GRAVITY = registers::GRAVITY_DATA_X_LSB
    };

public:
    // FUNCTION SECTION

    /**
     * @brief Construct a new imuAdafruitBNO055 object, which is responsible for communicating with Adafruit BNO005 sensor and startup i2c communication
     * @param sensorId sensor's id (in most cases it will be -1)
     * @param address sensor's address (can be checked with "gpio i2cdetect" function)
     */
    imuAdafruitBNO055(int32_t sensor_id = -1, uint8_t address = IMU_ADRESS);

    bool begin(imuOperatingMode mode = imuOperatingMode::NDOF);

    void setMode(imuOperatingMode mode);

    void getMode(imuOperatingMode &mode);

    void setAxisRemap(remapConfigs remapcode);

    void setAxisSign(remapSigns remapsign);

    void getRevInfo(revisions &rev);

    void setExtCrystalUse(bool usextal);

    void getSystemStatus(uint8_t &system_status, uint8_t &self_test_result, uint8_t &system_error);

    void getCalibration(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag);

    void getVector(int16_t (&vector)[3], vectorType vector_type);

    void getQuaternion(int16_t (&vector)[4]);

    void getTemperature(int8_t &temp);

    // bool getEvent(sensorEvent &event);

    // bool getEvent(sensorEvent &event, vectorType type);

    // void getSensor(sensorType &sensor);

    bool getSensorOffsets(uint8_t &calib_data);

    bool getSensorOffsets(imuOffsets &offset_type);

    void setSensorOffsets(const uint8_t callib_data);

    void setSensorOffsets(const imuOffsets offset_type);

    bool isFullyCalibrated();

    void enterSuspendMode();

    void enterNormalMode();

private:
    
    uint8_t readByte_(registers reg);
    
    bool readLen_(registers reg, uint8_t (&buffer)[], uint8_t len);
    
    bool writeByte_(registers reg, uint8_t value);

    int i2c_handle_;
    int32_t sensor_id_;
    imuOperatingMode mode_;
};


#endif