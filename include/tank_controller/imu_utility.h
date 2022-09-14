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
#include <iostream>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

// Define basic constants
#define IMU_ADRESS (0x28)
#define IMU_ID (0xA0)
#define IMU_OFFSET_REGISTERS (22)
#define IMU_SAMPLERATE_DELAY_MS (100)


namespace imu {


/**
 * @brief Wait for given ammount of time 
 * @param ms delay time [ms]
 */
inline void delay(uint ms) {
    #ifdef _WIN32
        Sleep(ms);         
    #else
        usleep(ms * 1000);
    #endif
}


/**
 * @brief Struct holding offset values for Adafruit BNO055 sensor
 */
struct Offsets {
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
enum OperatingMode {
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
class AdafruitBNO055 {
public:
    // DATA SECTION
    
    /**
     * @brief List of all registers that will be used when communicating with Adafruit BNO055 sensor
     */
    enum Registers {
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
    enum Powermodes {
        NORMAL = 0X00,
        LOWPOWER = 0X01,
        SUSPEND = 0X02
    };

    /**
     * @brief List of remap settings
     */
    enum RemapConfigs {
        CONFIG_P0 = 0x21,
        CONFIG_P1 = 0x24, // default
        CONFIG_P2 = 0x24,
        CONFIG_P3 = 0x21,
        CONFIG_P4 = 0x24,
        CONFIG_P5 = 0x21,
        CONFIG_P6 = 0x21,
        CONFIG_P7 = 0x24
    };

    /**
     * @brief List of remap signs
     */
    enum RemapSigns {
        SIGN_P0 = 0x04,
        SIGN_P1 = 0x00, // default
        SIGN_P2 = 0x06,
        SIGN_P3 = 0x02,
        SIGN_P4 = 0x03,
        SIGN_P5 = 0x01,
        SIGN_P6 = 0x07,
        SIGN_P7 = 0x05
    };

    /**
     * @brief Struct representing revisions
     */
    struct Revisions {
        uint8_t accelerometer;
        uint8_t magnetometer;
        uint8_t gyroscope;
        uint16_t sw;
        uint8_t bootloader;
    };

    /**
     * @brief List of useful vector mappings
     */
    enum VectorType {
        ACCELEROMETER = Registers::ACCEL_DATA_X_LSB,
        MAGNETOMETER = Registers::MAG_DATA_X_LSB,
        GYROSCOPE = Registers::GYRO_DATA_X_LSB,
        EULER = Registers::EULER_H_LSB,
        LINEARACCEL = Registers::LINEAR_ACCEL_DATA_X_LSB,
        GRAVITY = Registers::GRAVITY_DATA_X_LSB
    };

public:
    // FUNCTION SECTION

    /**
     * @brief Construct a new imuAdafruitBNO055 object, which is responsible for communicating with Adafruit BNO005 sensor and startup i2c communication
     * @param sensorId sensor's id (in most cases it will be -1)
     * @param address sensor's address (can be checked with "gpio i2cdetect" function)
     */
    AdafruitBNO055(int32_t sensor_id = -1, uint8_t address = IMU_ADRESS);

    /**
     * @brief Establish connection with device, reset it and switch it to given power mode
     * @param mode Mode the device will be set into
     * @return true if initialization was successfull
     * @return false if initialization failed
     */
    bool begin(OperatingMode mode = OperatingMode::NDOF);

    /**
     * @brief Put the chip in the specified operating mode
     * @param mode chosen operating mode
     */
    void setMode(OperatingMode mode);

    /**
     * @brief Get the current operating mode and write it into provided buffer
     * @param mode reference to a buffer which will hold received mode
     */
    void getMode(OperatingMode &mode);

    /**
     * @brief Change the chip's axis remap
     * @param remapcode  desired remap value
     */
    void setAxisRemap(RemapConfigs remapcode);

    /**
     * @brief Change the chip's axis sign
     * @param remapcsign  desired remap sign
     */
    void setAxisSign(RemapSigns remapsign);

    /**
     * @brief Get the chip revision numbers
     * @param rev reference to struct, to which revision info will be written into
     */
    void getRevInfo(Revisions &rev);

    /**
     * @brief Use the external 32.768KHz crystal
     * @param usextal use crystal
     */
    void setExtCrystalUse(bool usextal);
    
    /**
     * @brief Get the latest system status info
     * @param system_status Reference to write System Status info:  
     *  0 = Idle;  
     * 1 = System Error;  
     * 2 = Initializing Peripherals;  
     * 3 = System Iniitalization;  
     * 4 = Executing Self-Test;  
     * 5 = Sensor fusion algorithm running;  
     * 6 = System running without fusion algorithms
     * @param self_test_result Reference to write Self Test results:  
     *  1 = test passed, 0 = test failed;  
     *  Bit 0 = Accelerometer self test;  
     *  Bit 1 = Magnetometer self test;  
     *  Bit 2 = Gyroscope self test;  
     *  Bit 3 = MCU self test;  
     * @param system_error  Reference to write potential system errors
     *  0 = No error;  
     *  1 = Peripheral initialization error;  
     *  2 = System initialization error;  
     *  3 = Self test result failed;  
     *  4 = Register map value out of range;  
     *  5 = Register map address out of range;  
     *  6 = Register map write error;  
     *  7 = BNO low power mode not available for selected operat ion mode;  
     *  8 = Accelerometer power mode not available;  
     *  9 = Fusion algorithm configuration error;  
     *  A = Sensor configuration error;  
     */
    void getSystemStatus(uint8_t &system_status, uint8_t &self_test_result, uint8_t &system_error);

    /**
     * @brief Get the current calibration state. Each reference's value will be set to 0 if not calibrated or 3 if fully calibrated
     * @param system_status reference to which function will write current system calibration status, depends on all sensors
     * @param gyro reference to which function will write current calibration of gyroscope
     * @param accel reference to which function will write current calibration of accelerometer
     * @param mag reference to whichfunction will  write current calibration of magnetometer
     */
    void getCalibration(uint8_t &system_status, uint8_t &gyro, uint8_t &accel, uint8_t &mag);

    /**
     * @brief Get a vector reading from the specified source and write it to buffer array
     * @param vector array to store data into
     * @param vector_type type of data
     */
    void getVector(float vector[3], VectorType vector_type);

    /**
     * @brief Get a quaternion reading and write it to buffer array
     * @param vector array to store data into
     */
    void getQuaternion(float vector[4]);

    /**
     * @brief Get temperature in degress Celsius
     * @param temp reference to value in which temperature will be stored
     */
    void getTemperature(int8_t &temp);

    // bool getEvent(sensorEvent &event);

    // bool getEvent(sensorEvent &event, vectorType type);

    // void getSensor(sensorType &sensor);

    /**
     * @brief Reads the sensor's offset registers into a byte array
     * @param calib_data byte array to which data will be saved
     * @return true if read was successfull
     * @return false if read failed 
     */
    bool getSensorOffsets(uint8_t calib_data[IMU_OFFSET_REGISTERS]);

    /**
     * @brief Reads the sensor's offset registers into an offset struct
     * @param offset_type struct to which data will be saved
     * @return true if read was successfull
     * @return false if read failed
     */
    bool getSensorOffsets(Offsets &offset_type);

    /**
     * @brief Write an array of calibration values to the sensor's offset
     * @param callib_data calibration data
     */
    void setSensorOffsets(const uint8_t calib_data[IMU_OFFSET_REGISTERS]);

    /**
     * @brief Write to the sensor's offset registers from an offset struct
     * @param offset_type offset struct
     */
    void setSensorOffsets(const Offsets offset_type);

    /**
     * @brief Checks if all cal status values are set to 3 (fully calibrated)
     * @return true if fully calibrated
     * @return false if not fully calibrated
     */
    bool isFullyCalibrated();

    /**
     * @brief Enter suspend (sleep) mode 
     */
    void enterSuspendMode();

    /**
     * @brief Enter normal (awake) mode
     */
    void enterNormalMode();

private:
    /**
     * @brief Read byte from chosen register
     * @param reg register from which to read data
     * @return uint8_t register data
     */
    uint8_t readByte_(Registers reg);
    
    /**
     * @brief Read the specified number of bytes, starting from given register
     * @param reg starting register
     * @param buffer buffer to which data will be stored
     * @param len number of bytes to read
     * @return true if read successfully
     * @return false if error occured
     */
    bool readLen_(Registers reg, uint8_t buffer[], uint8_t len);
    
    /**
     * @brief Write given value to selected register
     * @param reg register to write into
     * @param value value to write
     * @return true if written successfully
     * @return false if errror occured
     */
    bool writeByte_(Registers reg, uint8_t value);

    int i2c_handle_;
    int32_t sensor_id_;
    OperatingMode mode_;
};


}   // namespace imu


#endif