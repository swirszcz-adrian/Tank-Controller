#include "tank_controller/i2c_bno055.h"
using namespace bno055;


// ####### Public methods #######


ConnectionBridge::ConnectionBridge(const OperationMode mode, const bool use_alt_address, const bool use_alt_bus) {
    #ifdef BNO055_PRINT_DEBUG
        std::cout << "\nDEBUG-INFO: Print-Debug mode has been enabled. This may lead to filling terminal window with potentially useless data"
                  << "To disable this mode comment out \"#define BNO055_PRINT_DEBUG\" instruction (file i2c_bno055.h line 14) and then recompile the code\n\n";
        std::cout << "DEBUG-INFO: Attempting to establish connection with Adafruit BNO055 sensor. This may take a while\n";
    #endif

    // Open i2c connection
    if (use_alt_bus) {
        i2c_handle_ = open(I2C_BUS_ALT.c_str(), O_RDWR);
    } else {
        i2c_handle_ = open(I2C_BUS.c_str(), O_RDWR);
    }
    
    // Set i2c device
    int address = use_alt_address ? BOARD_ADDRESS_ALT : BOARD_ADDRESS;
    if (ioctl(i2c_handle_, I2C_SLAVE, address) != 0) {
        std::stringstream stream;
        stream << std::hex << address;
        throw std::runtime_error("Failed to connect with sensor at address 0x" + stream.str() +"! Make sure the device is connected and right address is used");
    }
    #ifdef BNO055_PRINT_DEBUG
        std::stringstream stream;
        stream << std::hex << address;
        std::cout << "DEBUG-INFO: Sucessfully connected with device at address 0x" << stream.str() << "\n";
    #endif

    // Check whether we have actually connected with Adafruit BNO055 sensor (there could be other devices with same address)
    byte chip_id = 0x00;
    if (!i2cRead_(Registers::CHIP_ID_ADDR, chip_id) || chip_id != BOARD_ID) {   // This work because logic expressions are read from left to right
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to confirm board's ID, retrying in 1 second\n";
        #endif
        
        // Give device 1s to boot then retry
        delay(1000);
        if (!i2cRead_(Registers::CHIP_ID_ADDR, chip_id) || chip_id != BOARD_ID) {
            throw std::runtime_error("Couldn't confirm board's ID, it is possible that other device uses the same address as sensor");
        }
    }
    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Device's ID confirmed, restoring device's default settings\n";
    #endif

    // Switch device to config mode and reset
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    if (!i2cWrite_(Registers::SYS_TRIGGER_ADDR, 0x20)) {
        throw std::runtime_error("Failed to restore default settings!");
    }
    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Default settings restored, waiting for the device to restart\n";
    #endif

    // Wait for the device to reset (up to 3 sec)
    chip_id = 0x00;
    delay(530);
    for (int i = 0; i < 301; i++) {
        i2cRead_(Registers::CHIP_ID_ADDR, chip_id);
        if (chip_id == BOARD_ID) {
            break;
        }

        if (i < 300) {
            delay(10);
            continue;
        }
        throw std::runtime_error("Device took too long to boot after restart!");
    }
    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Restarting finished, returning to normal power mode\n";
    #endif

    // Return to normal power mode
    if (!i2cWrite_(Registers::PWR_MODE_ADDR, PowerMode::POWER_MODE_NORMAL)) { throw std::runtime_error("Failed to switch power mode!"); }
    delay(20);
    if (!i2cWrite_(Registers::PAGE_ID_ADDR, 0x00)) { throw std::runtime_error("Failed to switch power mode!"); }
    if (!i2cWrite_(Registers::SYS_TRIGGER_ADDR, 0x00)) { throw std::runtime_error("Failed to switch power mode!"); }
    delay(20);
    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Returned to normal power mode\n";
    #endif

    setMode(mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Connection with Adafruit BNO055 sensor has been successfully established\n";
    #endif
}


ConnectionBridge::~ConnectionBridge() {
    close(i2c_handle_);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Connection with Adafruit BNO055 board has been successfully closed\n";
    #endif
}


void ConnectionBridge::setMode(const OperationMode mode) {
    // If mode has been switched update internal variable and five device some time to update
    if (i2cWrite_(Registers::OPR_MODE_ADDR, mode)) {
        mode_ = mode;
        delay(30);
        
        #ifdef BNO055_PRINT_DEBUG
            std::cout << "DEBUG-INFO: Device's operating mode has been switched\n";
        #endif
    } else {
        throw std::runtime_error("Failed to switch device's operating mode!");
    }
}


bool ConnectionBridge::getMode(OperationMode &mode) {
    // Read data from register and if read succeeded update operating mode and return true
    byte buffer = 0x00;
    if (!i2cRead_(Registers::OPR_MODE_ADDR, buffer)) {
        // If read failed return false
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to read current operating mode\n";
        #endif

        return false;
    }

    mode_ = (OperationMode) buffer; // Update internal variable (just in case)
    mode = mode_;                   // Update reference's value
    return true;
}


void ConnectionBridge::setExtCrystalUse(const bool usextal) {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Change page ID to 0
    if (!i2cWrite_(Registers::PAGE_ID_ADDR, 0)) { throw std::runtime_error("Failed to change page ID!"); }

    // Write appropriate data to register
    if (usextal) {
        if (!i2cWrite_(Registers::SYS_TRIGGER_ADDR, 0x80)) {  throw std::runtime_error("Failed to use external crystal!");  }
    } else {
        if (!i2cWrite_(Registers::SYS_TRIGGER_ADDR, 0x00))  {  throw std::runtime_error("Failed to use internal crystal!");  }
    }
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Changed sensor's crystal oscilator\n";
    #endif
}


void ConnectionBridge::setAxisRemap(const RemapConfig remapcode) {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Remap
    if (!i2cWrite_(Registers::AXIS_MAP_CONFIG_ADDR, remapcode)) { throw std::runtime_error("Failed to set new axis remap config!"); }
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Sensor's axis remap config has been updated\n";
    #endif
}


void ConnectionBridge::setAxisSign(const RemapSigns remapsign) {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Remap
    if (!i2cWrite_(Registers::AXIS_MAP_SIGN_ADDR, remapsign)) { throw std::runtime_error("Failed to set new axis remap sign!"); }
    delay(10);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Sensor's axis remap sign has been updated\n";
    #endif
}


bool ConnectionBridge::getRevInfo(Revisions &rev) {
    // Revision's data is stored in six consecutive registers
    byte buffer[6] = {};
    if (!i2cRead_(Registers::ACCEL_REV_ID_ADDR, buffer, 6)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get revision data\n";
        #endif

        return false;
    }

    // Write data from buffer to reference
    rev.accelerometer = (int) buffer[0];
    rev.magnetometer = (int) buffer[1];
    rev.gyroscope = (int) buffer[2];
    rev.sw = ((int16_t) buffer[4] << 8) | buffer[3];
    rev.bootloader = (int) buffer[5];

    return true;
}


bool ConnectionBridge::getSystemStatus(byte &system_status, byte &self_test_result, byte &system_error) {
    // Set page ID to 0
    if (!i2cWrite_(Registers::PAGE_ID_ADDR, 0x00)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to set page ID to 0\n";
        #endif

        return false;
    }

    // Read data from appropriate registers into buffers
    byte temp_system_status = 0x00;
    byte temp_self_test_result = 0x00;
    byte temp_system_error = 0x00;
    if (!i2cRead_(Registers::SYS_STAT_ADDR, temp_system_status) || !i2cRead_(Registers::SELFTEST_RESULT_ADDR, temp_self_test_result) || !i2cRead_(Registers::SYS_ERR_ADDR, system_error)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to set read system status\n";
        #endif

        return false;
    }
    
    // If read succeeded write data to references
    system_status = temp_system_status;
    self_test_result = temp_self_test_result;
    system_error = temp_system_error;

    return true;
}


bool ConnectionBridge::getCalibration(byte &system_status, byte &gyro, byte &accel, byte &mag) {
    // Read data, then separate informations with bit-wise operations
    byte data = {};
    if (!i2cRead_(Registers::CALIB_STAT_ADDR, data)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get calibration data\n";
        #endif

        return false;
    }
    
    system_status = (data >> 6) & 0x03;
    gyro = (data >> 4) & 0x03;
    accel = (data >> 2) & 0x03;
    mag = data & 0x03;

    return true;
}


bool ConnectionBridge::isFullyCalibrated() {
    // Read calibration state
    byte system_status = 0;
    byte gyro = 0;
    byte accel = 0;
    byte mag = 0;
    if (!getCalibration(system_status, gyro, accel, mag)) { return false; } // No need for "#ifdef BNO055_PRINT_DEBUG" since getCalibration already has this clauzure

    // Check calibration state given current operating mode
    switch (mode_) {
        case OperationMode::OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OperationMode::OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OperationMode::OPERATION_MODE_GYRONLY:
        case OperationMode::OPERATION_MODE_M4G:     // No magnetometer calibration required
            return (gyro == 3);
        case OperationMode::OPERATION_MODE_ACCMAG:
        case OperationMode::OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OperationMode::OPERATION_MODE_ACCGYRO:
        case OperationMode::OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OperationMode::OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system_status == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}


bool ConnectionBridge::getTemperature(int &temp) {
    byte buffer = 0;
    if (!i2cRead_(Registers::TEMP_ADDR, buffer)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get temperature data\n";
        #endif

        return false;
    }

    temp = (int) buffer;
    return true;
}


bool ConnectionBridge::getVector(float vector[3], const VectorMappings vector_type) {
    // Create buffers
    byte buffer[6] = {};
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    // Read data
    if(!i2cRead_((Registers) vector_type, buffer, 6)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get vector data\n";
        #endif

        return false;
    }
    
    // Split vector data into values
    x = ((int16_t) buffer[1] << 8) | buffer[0];
    y = ((int16_t) buffer[3] << 8) | buffer[2];
    z = ((int16_t) buffer[5] << 8) | buffer[4];
    
    // Covert value to an appropriate range and write it to output array
    switch (vector_type) {
        case VectorMappings::VECTOR_MAGNETOMETER:
            // 1 uT = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorMappings::VECTOR_GYROSCOPE:
            // 1 dps = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorMappings::VECTOR_EULER:
            // 1 degree = 16 LSB
            vector[0] = ((float) x) / 16.0f;
            vector[1] = ((float) y) / 16.0f;
            vector[2] = ((float) z) / 16.0f;
            break;
        case VectorMappings::VECTOR_ACCELEROMETER:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
        case VectorMappings::VECTOR_LINEARACCEL:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
        case VectorMappings::VECTOR_GRAVITY:
            // 1 m/s^2 = 100 LSB
            vector[0] = ((float) x) / 100.0f;
            vector[1] = ((float) y) / 100.0f;
            vector[2] = ((float) z) / 100.0f;
            break;
    }

    return true;
}


bool ConnectionBridge::getQuaternion(float vector[4]) {
    // Create buffers
    byte buffer[8] = {};
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    int16_t w = 0;

    // Read data
    if(!i2cRead_(Registers::QUATERNION_DATA_W_LSB_ADDR, buffer, 8)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get quaternion data\n";
        #endif

        return false;
    }

    // Split data
    w = ((int16_t) buffer[1] << 8) | buffer[0];
    x = ((int16_t) buffer[3] << 8) | buffer[2];
    y = ((int16_t) buffer[5] << 8) | buffer[4];
    z = ((int16_t) buffer[7] << 8) | buffer[6];

    // Assign data to quaternion
    const float scale = (1.0f / (1 << 14));
    vector[0] = scale * x;
    vector[1] = scale * y;
    vector[2] = scale * z;
    vector[3] = scale * w;

    return true;
}


bool ConnectionBridge::getSensorOffsets(byte calib_data[NUM_OFFSET_REGISTERS]) {
    // Check if device is calibrated
    if (!isFullyCalibrated()) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Can't get offsets if device is not calibrated\n";
        #endif

        return false;
    }

    // Create a buffer
    byte buffer[NUM_OFFSET_REGISTERS] = {};

    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Try to read data to buffer
    if (!i2cRead_(Registers::ACCEL_OFFSET_X_LSB_ADDR, buffer, NUM_OFFSET_REGISTERS)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get offsets data\n";
        #endif

        return false;
    }

    // Return to previous operating mode
    setMode(prev_mode);

    // If succeeded move data from buffer to reference
    std::copy(buffer, buffer + NUM_OFFSET_REGISTERS, calib_data);

    return true;
}


bool ConnectionBridge::getSensorOffsets(Offsets &offset_type) {
    // Check if device is calibrated
    if (!isFullyCalibrated()) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Can't get offsets if device is not calibrated\n";
        #endif

        return false;
    }

    // Create a buffer
    byte buffer[NUM_OFFSET_REGISTERS] = {};

    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Try to read data to buffer
    if (!i2cRead_(Registers::ACCEL_OFFSET_X_LSB_ADDR, buffer, NUM_OFFSET_REGISTERS)) {
        #ifdef BNO055_PRINT_DEBUG
            std::cerr << "DEBUG-WARN: Failed to get offsets data\n";
        #endif

        return false;
    }

    // Return to previous operating mode
    setMode(prev_mode);

    // If succeeded split and move data from buffer to offset struct
    offset_type.accel_x = ((int16_t) buffer[1] << 8) | buffer[0];
    offset_type.accel_y = ((int16_t) buffer[3] << 8) | buffer[2];
    offset_type.accel_z = ((int16_t) buffer[5] << 8) | buffer[4];

    offset_type.mag_x = ((int16_t) buffer[7] << 8) | buffer[6];
    offset_type.mag_y = ((int16_t) buffer[9] << 8) | buffer[8];
    offset_type.mag_z = ((int16_t) buffer[11] << 8) | buffer[10];
    
    offset_type.gyro_x = ((int16_t) buffer[13] << 8) | buffer[12];
    offset_type.gyro_y = ((int16_t) buffer[15] << 8) | buffer[14];
    offset_type.gyro_z = ((int16_t) buffer[17] << 8) | buffer[16];

    offset_type.accel_radius = ((int16_t) buffer[19] << 8) | buffer[18];
    offset_type.mag_radius = ((int16_t) buffer[21] << 8) | buffer[20];

    return true;
}


void ConnectionBridge::setSensorOffsets(const byte calib_data[NUM_OFFSET_REGISTERS]) {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Try to read data to buffer
    if (!i2cWrite_(Registers::ACCEL_OFFSET_X_LSB_ADDR, calib_data, NUM_OFFSET_REGISTERS)) {
        throw std::runtime_error("Failed to update sensor's offsets!");
    }
    delay(25);

    // Return to previous operating mode
    setMode(prev_mode);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Sensor's offsets have been updated\n";
    #endif
}


void ConnectionBridge::setSensorOffsets(const Offsets offset_type) {
    // Write data from struct into buffer array
    byte buffer[NUM_OFFSET_REGISTERS] = {};
    buffer[0] = (offset_type.accel_x) & 0xFF;
    buffer[1] = (offset_type.accel_x >> 8) & 0xFF;
    buffer[2] = (offset_type.accel_y) & 0xFF;
    buffer[3] = (offset_type.accel_y >> 8) & 0xFF;
    buffer[4] = (offset_type.accel_z) & 0xFF;
    buffer[5] = (offset_type.accel_z >> 8) & 0xFF;

    buffer[6] = (offset_type.mag_x) & 0xFF;
    buffer[7] = (offset_type.mag_x >> 8) & 0xFF;
    buffer[8] = (offset_type.mag_y) & 0xFF;
    buffer[9] = (offset_type.mag_y >> 8) & 0xFF;
    buffer[10] = (offset_type.mag_z) & 0xFF;
    buffer[11] = (offset_type.mag_z >> 8) & 0xFF;

    buffer[12] = (offset_type.gyro_x) & 0xFF;
    buffer[13] = (offset_type.gyro_x >> 8) & 0xFF;
    buffer[14] = (offset_type.gyro_y) & 0xFF;
    buffer[15] = (offset_type.gyro_y >> 8) & 0xFF;
    buffer[16] = (offset_type.gyro_z) & 0xFF;
    buffer[17] = (offset_type.gyro_z >> 8) & 0xFF;

    buffer[18] = (offset_type.accel_radius) & 0xFF;
    buffer[19] = (offset_type.accel_radius >> 8) & 0xFF;
    buffer[20] = (offset_type.mag_radius) & 0xFF;
    buffer[21] = (offset_type.mag_radius >> 8) & 0xFF;    

    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    // Try to read data to buffer
    if (!i2cWrite_(Registers::ACCEL_OFFSET_X_LSB_ADDR, buffer, NUM_OFFSET_REGISTERS)) {
        throw std::runtime_error("Failed to update sensor's offsets!");
    }
    delay(25);

    // Return to previous operating mode
    setMode(prev_mode);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Sensor's offsets have been updated\n";
    #endif
}


void ConnectionBridge::enterSuspendPowerMode() {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    if (!i2cWrite_(Registers::PWR_MODE_ADDR, PowerMode::POWER_MODE_SUSPEND)) { throw std::runtime_error("Failed to change power mode!"); }
    delay(20);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Device's power mode has been changed to SUSPEND\n";
    #endif
}

void ConnectionBridge::enterNormalPowerMode() {
    // Backup previous operating mode and temporarily change to config mode
    OperationMode prev_mode = mode_;
    setMode(OperationMode::OPERATION_MODE_CONFIG);
    delay(25);

    if (!i2cWrite_(Registers::PWR_MODE_ADDR, PowerMode::POWER_MODE_NORMAL)) { throw std::runtime_error("Failed to change power mode!"); }
    delay(20);

    // Return to previous operating mode
    setMode(prev_mode);
    delay(20);

    #ifdef BNO055_PRINT_DEBUG
        std::cout << "DEBUG-INFO: Device's power mode has been changed to NORMAL\n";
    #endif
}


// ####### Private methods #######


bool ConnectionBridge::i2cWrite_(const Registers start_reg, const byte data[], const size_t len) {
    // To write multiple bytes you need to create an array (aka statically allocated meory space) consisting of starting register and rest of data
    byte write_buffer[len + 1] = {};
    write_buffer[0] = start_reg;
    std::copy(data, data + len, write_buffer + 1);  // This could also be done with "for" loop

    // Write data and check whether it succeeded
    if (write(i2c_handle_, write_buffer, len + 1) != (len + 1)) {
        #ifdef BNO055_PRINT_DEBUG
            fprintf(stderr, "DEBUG-WARN: Failed to write data to registers: 0x%.2X .. 0x%.2X\n", start_reg, start_reg + len);
        #endif
        return false;
    }
    
    return true;
}


bool ConnectionBridge::i2cWrite_(const Registers reg, const byte data) {
    // To write a single byte you need to create an array consisting of register to which write data and data itself
    byte write_buffer[2] = {reg, data};
    
    // Write data and check whether it succeeded
    if (write(i2c_handle_, write_buffer, 2) != 2) {
        #ifdef BNO055_PRINT_DEBUG
            fprintf(stderr, "DEBUG-WARN: Failed to write data to register: 0x%.2X\n", reg);
        #endif
        return false;
    }
    
    return true;
}


bool ConnectionBridge::i2cRead_(const Registers start_reg, byte buffer[], const size_t len) {
    // if (write(i2c_handle_, &start_reg, 1) != 1) {
    //     #ifdef BNO055_PRINT_DEBUG
    //         fprintf(stderr, "DEBUG-WARN: Failed to request read from registers: 0x%.2X .. 0x%.2X\n", start_reg, start_reg + len);
    //     #endif
    //     return false;
    // }

    // To read data you first need to write the register from which reading will start
    write(i2c_handle_, &start_reg, 1);

    // Read data and check whether operation succeeded
    if (read(i2c_handle_, buffer, len) != len) {
        #ifdef BNO055_PRINT_DEBUG
            fprintf(stderr, "DEBUG-WARN: Failed to read data from registers: 0x%.2X .. 0x%.2X\n", start_reg, start_reg + len);
        #endif
        return false;
    }

    return true;
}


bool ConnectionBridge::i2cRead_(const Registers reg, byte &buffer) {
    // if (write(i2c_handle_, &reg, 1) != 1) {
    //     #ifdef BNO055_PRINT_DEBUG
    //         fprintf(stderr, "DEBUG-WARN: Failed to request read from register: 0x%.2X \n", reg);
    //     #endif
    //     return false;
    // }

    // Write register from which data will be read
    write(i2c_handle_, &reg, 1);

    // Read data and check whether operation succeeded
    if (read(i2c_handle_, &buffer, 1) != 1) {
        #ifdef BNO055_PRINT_DEBUG
            fprintf(stderr, "DEBUG-WARN: Failed to read data from register: 0x%.2X \n", reg);
        #endif
        return false;
    }

    return true;
}