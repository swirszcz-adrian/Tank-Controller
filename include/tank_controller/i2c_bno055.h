#ifndef I2C_BNO055_H
#define I2C_BNO055_H


#include <iostream>
#include <algorithm>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>


// Warning: uncomenting the line below will fill your terminal window with debug messages
// #define BNO055_PRINT_DEBUG


namespace bno055 {
    // This way code will be less confusing
    using byte = uint8_t;


    // Basic board data
    const byte BOARD_ADDRESS = 0x28;
    const byte BOARD_ADDRESS_ALT = 0x29;
    const byte BOARD_ID = 0xA0;
    const unsigned int NUM_OFFSET_REGISTERS = 22;
    const std::string I2C_BUS = "/dev/i2c-1";
    const std::string I2C_BUS_ALT = "/dev/i2c-0";


    /**
     * @brief Wait for given ammount of time 
     * @param ms delay time [ms]
     */
    inline void delay(unsigned int ms) { usleep(ms * 1000U); }
    

    /**
     * @brief Struct holding offset values for Adafruit BNO055 sensor
     */
    struct Offsets {
        int accel_x;
        int accel_y;
        int accel_z;

        int mag_x;
        int mag_y;
        int mag_z;

        int gyro_x;
        int gyro_y;
        int gyro_z;
    
        int accel_radius;
       
        int mag_radius;
    };


    /**
     * @brief Struct representing revisions
     */
    struct Revisions {
        int accelerometer;
        int magnetometer;
        int gyroscope;
        int sw;
        int bootloader;
    };


    /**
     * @brief Enumeration for storing Adafruit BNO005 sensor's operating modes
     */
    enum OperationMode{
        OPERATION_MODE_CONFIG = 0x00,
        OPERATION_MODE_ACCONLY = 0x01,
        OPERATION_MODE_MAGONLY = 0x02,
        OPERATION_MODE_GYRONLY = 0x03,
        OPERATION_MODE_ACCMAG = 0x04,
        OPERATION_MODE_ACCGYRO = 0x05,
        OPERATION_MODE_MAGGYRO = 0x06,
        OPERATION_MODE_AMG = 0x07,
        OPERATION_MODE_IMUPLUS = 0x08,
        OPERATION_MODE_COMPASS = 0x09,
        OPERATION_MODE_M4G = 0x0A,
        OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
        OPERATION_MODE_NDOF = 0x0C
    };


    /**
     * @brief List of sensor's registers
     */
    enum Registers{
        // Page id register definition 
        PAGE_ID_ADDR = 0X07,

        // PAGE0 REGISTER DEFINITION START
        CHIP_ID_ADDR = 0x00,
        ACCEL_REV_ID_ADDR = 0x01,
        MAG_REV_ID_ADDR = 0x02,
        GYRO_REV_ID_ADDR = 0x03,
        SW_REV_ID_LSB_ADDR = 0x04,
        SW_REV_ID_MSB_ADDR = 0x05,
        BL_REV_ID_ADDR = 0X06,

        // Accel data register
        ACCEL_DATA_X_LSB_ADDR = 0X08,
        ACCEL_DATA_X_MSB_ADDR = 0X09,
        ACCEL_DATA_Y_LSB_ADDR = 0X0A,
        ACCEL_DATA_Y_MSB_ADDR = 0X0B,
        ACCEL_DATA_Z_LSB_ADDR = 0X0C,
        ACCEL_DATA_Z_MSB_ADDR = 0X0D,

        // Mag data register
        MAG_DATA_X_LSB_ADDR = 0X0E,
        MAG_DATA_X_MSB_ADDR = 0X0F,
        MAG_DATA_Y_LSB_ADDR = 0X10,
        MAG_DATA_Y_MSB_ADDR = 0X11,
        MAG_DATA_Z_LSB_ADDR = 0X12,
        MAG_DATA_Z_MSB_ADDR = 0X13,

        // Gyro data registers
        GYRO_DATA_X_LSB_ADDR = 0X14,
        GYRO_DATA_X_MSB_ADDR = 0X15,
        GYRO_DATA_Y_LSB_ADDR = 0X16,
        GYRO_DATA_Y_MSB_ADDR = 0X17,
        GYRO_DATA_Z_LSB_ADDR = 0X18,
        GYRO_DATA_Z_MSB_ADDR = 0X19,

        // Euler data registers
        EULER_H_LSB_ADDR = 0X1A,
        EULER_H_MSB_ADDR = 0X1B,
        EULER_R_LSB_ADDR = 0X1C,
        EULER_R_MSB_ADDR = 0X1D,
        EULER_P_LSB_ADDR = 0X1E,
        EULER_P_MSB_ADDR = 0X1F,

        // Quaternion data registers
        QUATERNION_DATA_W_LSB_ADDR = 0X20,
        QUATERNION_DATA_W_MSB_ADDR = 0X21,
        QUATERNION_DATA_X_LSB_ADDR = 0X22,
        QUATERNION_DATA_X_MSB_ADDR = 0X23,
        QUATERNION_DATA_Y_LSB_ADDR = 0X24,
        QUATERNION_DATA_Y_MSB_ADDR = 0X25,
        QUATERNION_DATA_Z_LSB_ADDR = 0X26,
        QUATERNION_DATA_Z_MSB_ADDR = 0X27,

        // Linear acceleration data registers
        LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
        LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
        LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
        LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
        LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
        LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

        // Gravity data registers
        GRAVITY_DATA_X_LSB_ADDR = 0X2E,
        GRAVITY_DATA_X_MSB_ADDR = 0X2F,
        GRAVITY_DATA_Y_LSB_ADDR = 0X30,
        GRAVITY_DATA_Y_MSB_ADDR = 0X31,
        GRAVITY_DATA_Z_LSB_ADDR = 0X32,
        GRAVITY_DATA_Z_MSB_ADDR = 0X33,

        // Temperature data register
        TEMP_ADDR = 0X34,

        // Status registers
        CALIB_STAT_ADDR = 0X35,
        SELFTEST_RESULT_ADDR = 0X36,
        INTR_STAT_ADDR = 0X37,
        SYS_CLK_STAT_ADDR = 0X38,
        SYS_STAT_ADDR = 0X39,
        SYS_ERR_ADDR = 0X3A,

        // Unit selection register
        UNIT_SEL_ADDR = 0X3B,

        // Mode registers
        OPR_MODE_ADDR = 0X3D,
        PWR_MODE_ADDR = 0X3E,
        SYS_TRIGGER_ADDR = 0X3F,
        TEMP_SOURCE_ADDR = 0X40,

        // Axis remap registers
        AXIS_MAP_CONFIG_ADDR = 0X41,
        AXIS_MAP_SIGN_ADDR = 0X42,

        // SIC registers
        SIC_MATRIX_0_LSB_ADDR = 0X43,
        SIC_MATRIX_0_MSB_ADDR = 0X44,
        SIC_MATRIX_1_LSB_ADDR = 0X45,
        SIC_MATRIX_1_MSB_ADDR = 0X46,
        SIC_MATRIX_2_LSB_ADDR = 0X47,
        SIC_MATRIX_2_MSB_ADDR = 0X48,
        SIC_MATRIX_3_LSB_ADDR = 0X49,
        SIC_MATRIX_3_MSB_ADDR = 0X4A,
        SIC_MATRIX_4_LSB_ADDR = 0X4B,
        SIC_MATRIX_4_MSB_ADDR = 0X4C,
        SIC_MATRIX_5_LSB_ADDR = 0X4D,
        SIC_MATRIX_5_MSB_ADDR = 0X4E,
        SIC_MATRIX_6_LSB_ADDR = 0X4F,
        SIC_MATRIX_6_MSB_ADDR = 0X50,
        SIC_MATRIX_7_LSB_ADDR = 0X51,
        SIC_MATRIX_7_MSB_ADDR = 0X52,
        SIC_MATRIX_8_LSB_ADDR = 0X53,
        SIC_MATRIX_8_MSB_ADDR = 0X54,

        // Accelerometer Offset registers
        ACCEL_OFFSET_X_LSB_ADDR = 0X55,
        ACCEL_OFFSET_X_MSB_ADDR = 0X56,
        ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
        ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
        ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
        ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

        // Magnetometer Offset registers
        MAG_OFFSET_X_LSB_ADDR = 0X5B,
        MAG_OFFSET_X_MSB_ADDR = 0X5C,
        MAG_OFFSET_Y_LSB_ADDR = 0X5D,
        MAG_OFFSET_Y_MSB_ADDR = 0X5E,
        MAG_OFFSET_Z_LSB_ADDR = 0X5F,
        MAG_OFFSET_Z_MSB_ADDR = 0X60,

        // Gyroscope Offset registers
        GYRO_OFFSET_X_LSB_ADDR = 0X61,
        GYRO_OFFSET_X_MSB_ADDR = 0X62,
        GYRO_OFFSET_Y_LSB_ADDR = 0X63,
        GYRO_OFFSET_Y_MSB_ADDR = 0X64,
        GYRO_OFFSET_Z_LSB_ADDR = 0X65,
        GYRO_OFFSET_Z_MSB_ADDR = 0X66,

        // Radius registers
        ACCEL_RADIUS_LSB_ADDR = 0X67,
        ACCEL_RADIUS_MSB_ADDR = 0X68,
        MAG_RADIUS_LSB_ADDR = 0X69,
        MAG_RADIUS_MSB_ADDR = 0X6A
    };


    /**
     * @brief List of sensor's power settings
     */
    enum PowerMode{
        POWER_MODE_NORMAL = 0X00,
        POWER_MODE_LOWPOWER = 0X01,
        POWER_MODE_SUSPEND = 0X02
    };


    /**
     * @brief List of remap settings
     */
    enum RemapConfig{
        REMAP_CONFIG_P0 = 0x21,
        REMAP_CONFIG_P1 = 0x24, // default
        REMAP_CONFIG_P2 = 0x24,
        REMAP_CONFIG_P3 = 0x21,
        REMAP_CONFIG_P4 = 0x24,
        REMAP_CONFIG_P5 = 0x21,
        REMAP_CONFIG_P6 = 0x21,
        REMAP_CONFIG_P7 = 0x24
    };


    /**
     * @brief List of remap signs
     */
    enum RemapSigns{
        REMAP_SIGN_P0 = 0x04,
        REMAP_SIGN_P1 = 0x00, // default
        REMAP_SIGN_P2 = 0x06,
        REMAP_SIGN_P3 = 0x02,
        REMAP_SIGN_P4 = 0x03,
        REMAP_SIGN_P5 = 0x01,
        REMAP_SIGN_P6 = 0x07,
        REMAP_SIGN_P7 = 0x05
    };


    /**
     * @brief List of useful vector mappings
     */
    enum VectorMappings{
        VECTOR_ACCELEROMETER = (int) Registers::ACCEL_DATA_X_LSB_ADDR,
        VECTOR_MAGNETOMETER = (int) Registers::MAG_DATA_X_LSB_ADDR,
        VECTOR_GYROSCOPE = (int) Registers::GYRO_DATA_X_LSB_ADDR,
        VECTOR_EULER = (int) Registers::EULER_H_LSB_ADDR,
        VECTOR_LINEARACCEL = (int) Registers::LINEAR_ACCEL_DATA_X_LSB_ADDR,
        VECTOR_GRAVITY = (int) Registers::GRAVITY_DATA_X_LSB_ADDR
    };


    /**
     * @brief Class containing all the necessary methods to interface with Adafruit BNO005 IMU sensor via I2C protocol
     */
    class ConnectionBridge {
    public:
        /**
         * @brief Construct a new ConnectionBridge object and initialize i2c connection with board. Execution will take a few seconds since program has to wait for board to respond
         * @param mode sensor's operating mode
         * @param use_alt_address whether to use default or alternative board address
         * @param use_alt_bus whether to use default or alternative i2c bus
         * @throws std::runtime_error if connection with board couldn't be established
         */
        ConnectionBridge(const OperationMode mode = OperationMode::OPERATION_MODE_NDOF, const bool use_alt_address = false, const bool use_alt_bus = false);

        /**
         * @brief Destroy the ConnectionBridge object and close i2c connection
         */
        ~ConnectionBridge();

        /**
         * @brief Put the chip in the specified operating mode
         * @param mode chosen operating mode
         * @throws std::runtime_error if setting failed
         */
        void setMode(const OperationMode mode);

        /**
         * @brief Get the current operating mode and write it into provided buffer
         * @param mode reference to a buffer which will hold received mode
         * @return true if succesfully read operating mode
         * @return false if failed to read operating mode
         */
        bool getMode(OperationMode &mode);

        /**
         * @brief Use the external 32.768KHz crystal
         * @param usextal use crystal
         * @throws std::runtime_error if setting failed
         */
        void setExtCrystalUse(const bool usextal);

        /**
         * @brief Change the chip's axis remap
         * @param remapcode  desired remap value
         * @throws std::runtime_error if setting failed
         */
        void setAxisRemap(const RemapConfig remapcode);

        /**
         * @brief Change the chip's axis sign
         * @param remapcsign  desired remap sign
         * @throws std::runtime_error if setting failed
         */
        void setAxisSign(const RemapSigns remapsign);

        /**
         * @brief Get the chip revision numbers
         * @param rev reference to struct, to which revision info will be written into
         * @return true if successfully got all revision data
         * @return false if failed to get revision data
         */
        bool getRevInfo(Revisions &rev);

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
         * @return true if got all data
         * @return false if failed to get all data
         */
        bool getSystemStatus(byte &system_status, byte &self_test_result, byte &system_error);

        /**
         * @brief Get the current calibration state. Each reference's value will be set to 0 if not calibrated or 3 if fully calibrated
         * @param system_status reference to which function will write current system calibration status, depends on all sensors
         * @param gyro reference to which function will write current calibration of gyroscope
         * @param accel reference to which function will write current calibration of accelerometer
         * @param mag reference to whichfunction will  write current calibration of magnetometer
         * @return true if got all data
         * @return false if failed to get all data
         */
        bool getCalibration(byte &system_status, byte &gyro, byte &accel, byte &mag);

        /**
         * @brief Checks if all cal status values are set to 3 (fully calibrated)
         * @return true if fully calibrated
         * @return false if not fully calibrated or failed to get calibration data
         */
        bool isFullyCalibrated();

        /**
         * @brief Get temperature in degress Celsius
         * @param temp reference to value in which temperature will be stored
         * @return true if got temparature data
         * @return false if failed to get temperature data
         */
        bool getTemperature(int &temp);

        /**
         * @brief Get a vector reading from the specified source and write it to buffer array
         * @param vector array to store data into
         * @param vector_type type of data
         * @return true if got requested data
         * @return false if failed to get data
         */
        bool getVector(float vector[3], const VectorMappings vector_type);

        /**
         * @brief Get a quaternion reading and write it to buffer array
         * @param vector array to store data into
         * @return true if got requested data
         * @return false if failed to get data
         */
        bool getQuaternion(float vector[4]);

        /**
         * @brief Reads the sensor's offset registers into a byte array.
         * You can only read offsets from fully calibrated sensor!
         * @param calib_data byte array to which data will be saved
         * @return true if read was successfull
         * @return false if read failed 
         * @throws std:runtime_error if changing operating mode fails (changing of operating mode is necessary to read offsets data)
         */
        bool getSensorOffsets(byte calib_data[NUM_OFFSET_REGISTERS]);

        /**
         * @brief Reads the sensor's offset registers into an offset struct.
         * You can only read offsets from fully calibrated sensor!
         * @param offset_type struct to which data will be saved
         * @return true if read was successfull
         * @return false if read failed
         * @throws std:runtime_error if changing operating mode fails (changing of operating mode is necessary to read offsets data)
         */
        bool getSensorOffsets(Offsets &offset_type);

        /**
         * @brief Write an array of calibration values to the sensor's offset
         * @param callib_data calibration data
         * @throws std:runtime_error if writing calibration data fails
         */
        void setSensorOffsets(const byte calib_data[NUM_OFFSET_REGISTERS]);

        /**
         * @brief Write to the sensor's offset registers from an offset struct
         * @param offset_type offset struct
         * @throws std:runtime_error if writing calibration data fails
         */
        void setSensorOffsets(const Offsets offset_type);

        /**
         * @brief Enter suspend (sleep) power mode 
         * @throws std:runtime_error if changing power mode fails
         */
        void enterSuspendPowerMode();

        /**
         * @brief Enter normal (awake) power mode
         * @throws std:runtime_error if changing power mode fails
         */
        void enterNormalPowerMode();


    private:
        /**
         * @brief Write given ammount of bytes to board's registers, starting from given register (and going up by one for each next byte)
         * @param start_reg register from which to start writing data
         * @param data array containing bytes to write
         * @param len number of bytes to write
         * @return true if write was successfull
         * @return false if write failed
         */
        bool i2cWrite_(const Registers start_reg, const byte data[], const size_t len);

        /**
         * @brief write a single byte of data to given register
         * @param reg register to which write data
         * @param data byte of data
         * @return true if write was successfull
         * @return false if write failed
         */
        bool i2cWrite_(const Registers reg, const byte data);
        
        /**
         * @brief Read given ammount of bytes from board's registers, starting from given register (and going up by one for each next byte)
         * @param start_reg register from which to start reading data
         * @param buffer buffer array to which data will be saved
         * @param len number of bytes to read
         * @return true if read was successfull
         * @return false if read failed
         */
        bool i2cRead_(const Registers start_reg, byte buffer[], const size_t len);

        /**
         * @brief Read a single byte from given register
         * @param reg register from which to read data
         * @param buffer reference to variable to which data will be saved
         * @return true if read was successfull
         * @return false if read failed
         */
        bool i2cRead_(const Registers reg, byte &buffer);


        int i2c_handle_;
        OperationMode mode_;
    };


}   // namespace bno055


#endif