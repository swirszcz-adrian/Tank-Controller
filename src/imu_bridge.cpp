// ####### WARNING #######
// THIS NODE CAN ONLY BE RUN ON RASPBERRY PI!


#if !defined(__arm__) || defined(_WIN32)
/* 
 * The "if" clauzure above checks if device has ARM archtecture and if it isn't running on Windows
 * Although this is far from foolproof, it's a simple solution to check whether device has capability to actually run the code
 */


#include <iostream>


int main() {
    std::cout << "ERROR: This program can only be run on Raspberry Pi, preferably with Raspbian OS" << std::ebdl;

    return -1;
}



#else
/*
 * The code below will only be executed, if the device has ARM archtecture and isn't running on Windows
 */


#include "ros/ros.h"
#include "tank_controller/i2c_bno055.h"
#include "tank_controller/imuData.h"
#include "tank_controller/imuResetVariables.h"

#include <iostream>
#include <memory>

using namespace bno055;

/**
 * @brief Simple class for holding vecotr data
 * 
 */
struct Vector {
    // Construct new vector from three floats
    Vector(float new_x = 0.0f, float new_y = 0.0f, float new_z = 0.0f) 
          : x(new_x)
          , y(new_y)
          , z(new_z) {}

    // Construct new vector from other vector
    Vector(const Vector &other) 
          : x(other.x)
          , y(other.y)
          , z(other.z) {}

    // Copy value's from other vector
    Vector& operator=(const Vector &other) {
        x = other.x;
        y = other.y;
        z = other.z; 

        return *this;
    }

    // Vector scaling
    Vector operator*(const float k) {
        x *= k;
        y *= k;
        z *= k;

        return *this;
    }

    // Vector addition
    Vector operator+(const Vector &other) {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }

    // Vector subtraction
    Vector operator-(const Vector &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }

    // Set all values to zero
    void setZero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    float x;
    float y;
    float z;
};


/**
 * @brief Class responsible for reading data from Adafruit BNO055 IMU sensor, converting it to linear and angular velocity and publishing it into "/imu_data" topic
 */
class ImuBridge {
private:
    /**
     * @brief Function called only inside constructor initializer list to avoid problems with ROS
     * @param node_name a string containings node's name
     * @return node_name (the same string you passed as a parameter)
     */
    std::string rosInit(std::string node_name) {
        // Initialize ROS without any remapping arguments (node is quite simple so we won't need them)
        ros::init(ros::M_string(), node_name);
        return node_name;
    }


public:
    /**
     * @brief Construct a new ImuBridge objectm responsible for processing Adafruit BNO055 sensor data and parsing it to "/imu_data" topic
     * @param loop_rate rate at which data should be published
     */
    ImuBridge(int loop_rate = 30) 
             : node_name_(ImuBridge::rosInit("imu_bridge")) 
             , nh_(ros::NodeHandle())
             , read_rate_(100)  // This value depends on sensor's update rate. For Adafruit BNO055 it is 100 Hz
             , loop_rate_(loop_rate)
             , linear_calib_(0)
             , angular_calib_(0)
             , linear_filtered_()
             , linear_prev_()
             , angular_filtered_()
             , angular_prev_()
             , gravity_()
             , accel_prev_() {
        ROS_INFO("Starting up %s node:", node_name_.c_str());

        // Create service server, that receives requests from /imu_reset_variables topic and passes them to
        // function resetVariablesServer, which is a method of ImuBridge class and belongs to "this" object 
        reset_vars_ser_ = nh_.advertiseService("/imu_reset_variables", &ImuBridge::resetVariablesServer, this);
        ROS_INFO(" - /global_stop service created");

        // Create publisher that sends messages of type tank_controller/imuData to /imu_data topic with message queue size equal to 1
        imu_data_pub_ = nh_.advertise<tank_controller::imuData>("/imu_data", 1);
        ROS_INFO(" - /imu_data publisher created");

        // Connect with IMU sensor
        // unique_ptr is used here to prevent program from trying to call ConnectionBridge's default constructor during declaration
        ROS_INFO(" - attempting to connect with Adafruit BNO055 sensor. This may take a while..");
        bno_ptr_ = std::unique_ptr<ConnectionBridge> {new ConnectionBridge(OperationMode::OPERATION_MODE_IMUPLUS)};
        bno_ptr_->setExtCrystalUse(true);
        ROS_INFO(" - connection with Adafruit BNO055 sensor has been established");

        ROS_INFO("Initialization finished");
    }

    // Remove copy constructor and assignement operator since we won't need them
    ImuBridge(const ImuBridge &) = delete;
    ImuBridge& operator=(const ImuBridge &) = delete;

    /**
     * @brief Destroy the ImuBridge object and print message
     */
    ~ImuBridge() {
        std::cout << "|===== Node has been successfully destroyed =====" << std::endl;
    }

    /**
     * @brief Function called automatically whenever new imuResetVariables request is received by service server.
     * There is no need for user to call this function manually!
     * @param req reference to request message, passed automatically by ROS
     * @param res reference to response message, passed automatically by ROS
     * @return true (always)
     */
    bool resetVariablesServer(tank_controller::imuResetVariables::Request &req, tank_controller::imuResetVariables::Response &res) {
        // Reset internal variables
        this->resetVariables_();

        ROS_WARN("Internal variables have been zeroed");
        return true;
    }

    /**
     * @brief Function called automatically by ros::Timer in regular time intervals to read and process data from sensor
     * There is no need for user to call this function manually!
     * @param event Struct holding some basic time-oriented data, passed by ros
     */
    void readSensorData(const ros::TimerEvent &event) {     
        // Read calibration data
        uint8_t sys_stat_calib = 0U;
        uint8_t mag_calib = 0U;
        bno_ptr_->getCalibration(sys_stat_calib, angular_calib_, linear_calib_, mag_calib);

        // Create buffers to which data will be written into
        float angular_buffer[3] = {0.0f, 0.0f, 0.0f};
        float linear_buffer[3] = {0.0f, 0.0f, 0.0f};
        float gravity_buffer[3] = {0.0f, 0.0f, 0.0f};

        // Create variables to which scaled data will be written into
        Vector angular = {};
        Vector linear = {};

        // Update gravity data
        if (bno_ptr_->getVector(angular_buffer, VectorMappings::VECTOR_GRAVITY)) {
            // Assign correct values and multiply some of them by -1
            gravity_.x = gravity_buffer[0];
            gravity_.y = -gravity_buffer[1];
            gravity_.z = -gravity_buffer[2];
        }

        // Read angular data. If read didn't succeeed, use data from previous loop
        if (bno_ptr_->getVector(angular_buffer, VectorMappings::VECTOR_GYROSCOPE)) {
            // For angular data we only need to assign correct values to correct variables
            angular.x = -angular_buffer[0];
            angular.y = angular_buffer[1];
            angular.z = angular_buffer[2];
        } else {
            angular = angular_prev_;
        }

        // Read linear data. If read didn't succeeed, use data from previous loop
        if (bno_ptr_->getVector(linear_buffer, VectorMappings::VECTOR_ACCELEROMETER)) {
            // For linear data we need to assign correct values to correct variables and subtract gravity
            linear.x = linear_buffer[0] - gravity_.x;
            linear.y = -linear_buffer[1] - gravity_.y;
            linear.z = -linear_buffer[2] - gravity_.z;

            // Update previous acceleration
            // This wasn't done in angular velocity, since back then we didn't need two vectors for storing old data
            // (in contrast, here we need one vector for storing acceleration, in case we can't get a read from sensor, and one for storing velocity)
            accel_prev_ = linear;
        } else {
            linear = accel_prev_;
        }

        // Caluclate the time between this and last loop
        float time_diff = (event.current_real - event.last_real).toSec();

        // Calculate velocity (typical discrete integral)
        linear = (linear * time_diff) + linear_prev_;

        // Low pass filter
        angular_filtered_ = angular_filtered_ * 0.63946f + angular * 0.18027f + angular_prev_ * 0.18027f;
        linear_filtered_ = linear_filtered_ * 0.63946f + linear * 0.18027f + linear_prev_ * 0.18027f;

        // Update values
        angular_prev_ = angular;
        linear_prev_ = linear;
    }

    /**
     * @brief Function called automatically by ros::Timer in regular time intervals to publish processed data to /imu_data topic
     * There is no need for user to call this function manually!
     * @param event Struct holding some basic time-oriented data, passed by ros
     */
    void publishSensorData(const ros::TimerEvent &event) {
        // Create message and fill it with basic data
        tank_controller::imuData msg;
        msg.header.stamp = ros::Time::now();

        // Fill calibration data
        msg.linearCalib = linear_calib_;
        msg.angularCalib = angular_calib_;

        // Fill velocity data
        msg.linear.x = linear_filtered_.x;
        msg.linear.y = linear_filtered_.y;
        msg.linear.z = linear_filtered_.z;

        msg.angular.x = angular_filtered_.x;
        msg.angular.y = angular_filtered_.y;
        msg.angular.z = angular_filtered_.z;

        // Publish message
        imu_data_pub_.publish(msg);
    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        // Since this node has to use two, different rates (one to read data from sensor and second to publish it) I've decided to use ros timers instead of typical "while" loop
        // Timers call given function in (somewhat) regular time intervals and pass to them structs, containing basic time-orientet data (like current time and time of last call)
        // Different timers can use different rates, making them perfect for this specific node

        ROS_INFO("Starting up main program loop");

        // Create timer, that calls function readSensorData, which is a method of ImuBridge class and belongs to "this" object
        // Period between calls will be roughly equal to 1/read_rate_
        read_timer_ = nh_.createTimer(ros::Duration(1/read_rate_), &ImuBridge::readSensorData, this);

        // Create timer, that calls function publishSensorData, which is a method of ImuBridge class and belongs to "this" object
        // Period between calls will be roughly equal to 1/loop_rate_
        publish_timer_ = nh_.createTimer(ros::Duration(1/loop_rate_), &ImuBridge::publishSensorData, this);

        // Reset internal variables (juuust in case)
        this->resetVariables_();

        ROS_INFO("Main program loop has beem started");
        ros::spin();    // Since everything is managed automatically there is no need for "while" loop

        return EXIT_SUCCESS;
    }

    /**
     * @brief Another way to call run() method (aka. main program loop).
     * @return EXIT_SUCCESS if closed properly
     */
    int operator()() {
        return this->run();
    }


private:
    /**
     * @brief Resets nodes internal variables (that is integrals, filters etc.)
     */
    void resetVariables_() {
        linear_calib_ = 0U;
        angular_calib_ = 0U;

        linear_filtered_.setZero();
        linear_prev_.setZero();
        angular_filtered_.setZero();
        angular_prev_.setZero();

        gravity_.setZero();
        accel_prev_.setZero();
    }

    std::string node_name_;
    
    uint8_t linear_calib_;
    uint8_t angular_calib_;
    Vector linear_filtered_;
    Vector linear_prev_;
    Vector angular_filtered_;
    Vector angular_prev_;
    
    Vector gravity_;
    Vector accel_prev_;

    std::unique_ptr<ConnectionBridge> bno_ptr_;

    ros::NodeHandle nh_;
    int read_rate_;
    int loop_rate_;
    ros::ServiceServer reset_vars_ser_;
    ros::Publisher imu_data_pub_;
    ros::Timer read_timer_;
    ros::Timer publish_timer_;
};


int main() {
    ImuBridge imu_bridge = {};
    return imu_bridge.run();
}

//     ConnectionBridge bno = {OperationMode::OPERATION_MODE_IMUPLUS};
//     bno.setExtCrystalUse(true);
//     float accel_vec[3] = {};
//     float gyro_vec[3] = {};
//     byte sys_stat = 0;
//     byte gyro = 0;
//     byte accel = 0;
//     byte mag = 0;
//     bool fully_calib = false;

//     while (true) {
//         bno.getVector(accel_vec, VectorMappings::VECTOR_ACCELEROMETER);
//         bno.getVector(gyro_vec, VectorMappings::VECTOR_GRAVITY); 
//         // bno.getCalibration(sys_stat, gyro, accel, mag);
//         fully_calib = bno.isFullyCalibrated();
//             std::cout << std::fixed << std::setw(7) << std::setprecision(2) 
//                       << "Accel:\n"
//                       << "  x = " << accel_vec[0] << "\n"
//                       << "  y = " << accel_vec[1] << "\n"
//                       << "  z = " << accel_vec[2] << "\n"
//                       << "Gyro:\n"
//                       << "  x = " << gyro_vec[0] << "\n"
//                       << "  y = " << gyro_vec[1] << "\n"
//                       << "  z = " << gyro_vec[2] << "\n" 
//                       << std::endl;
//                     //   << "Calib (" << (fully_calib ? "is" : "isn't") << " fully calibrated):\n"
//                     //   << "  acc = " << (int) accel << "\n"
//                     //   << "  gyr = " << (int) gyro << "\n"
//                     //   << "  mag = " << (int) mag << "\n" << std::endl;
//         bno055::delay(50);
//     }
// }


#endif