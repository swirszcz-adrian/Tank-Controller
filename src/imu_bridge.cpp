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
#include <signal.h>
#include <thread>


using namespace bno055;

#define DEG_2_RAD 0.01745329251


/**
 * @brief Function called when Ctrl+C is passed
 * @param sig sigint
 */
void rosSigintHandler(int sig) {
    ROS_INFO("Shutting down threads");
    ros::shutdown();
}


/**
 * @brief Simple class for holding vecotr data
 * 
 */
struct Vector {
    // Construct new vector from three floats
    Vector(double new_x = 0.0, double new_y = 0.0, double new_z = 0.0) 
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
    Vector operator*(const double k) {
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
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    double x;
    double y;
    double z;
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
        // Initialize ROS without any remapping arguments (node is quite simple so they won't be needed)
        ros::init(ros::M_string(), node_name);
        return node_name;
    }


public:
    /**
     * @brief Construct a new ImuBridge object responsible for processing Adafruit BNO055 sensor data and parsing it to "/imu_data" topic
     * @param loop_rate rate at which data should be published
     */
    ImuBridge(int loop_rate = 30) 
             : node_name_(ImuBridge::rosInit("imu_bridge")) 
             , nh_(ros::NodeHandle())
             , read_wait_(10)  // This value depends on sensor's update rate. For Adafruit BNO055 it is 100 Hz (this translates to aprrox. 10ms of wait time between consecutive reads)
             , publish_rate_(loop_rate)
             , accel_calib_(0U)
             , gyro_calib_(0U)
             , accel_()
             , time_prev_(0)
             , velocity_()
             , velocity_prev_()
             , velocity_filtered_()
             , rotation_()  {
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

    // Remove copy constructor and assignement operator since they won't be needed
    ImuBridge(const ImuBridge &) = delete;
    ImuBridge& operator=(const ImuBridge &) = delete;

    /**
     * @brief Destroy the ImuBridge object and print message
     */
    ~ImuBridge() {
        std::cout << "|===== Node has been successfully destroyed =====|" << std::endl;
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
    void readSensorData() {     
        // Read calibration data1C:A0:B8:01:CE:F2
        uint8_t sys_stat_calib = 0U;
        uint8_t mag_calib = 0U;
        bno_ptr_->getCalibration(sys_stat_calib, gyro_calib_, accel_calib_, mag_calib);

        // Read rotation data and (if read succeeded) scale it accordingly
        float rotation_buffer[3] = {0.0f, 0.0f, 0.0f};
        if(bno_ptr_->getVector(rotation_buffer, VectorMappings::VECTOR_EULER)) {
            // For rotation vector assign correct values to correct variables and scale them from degrees to radians
            rotation_.x = rotation_buffer[2] * DEG_2_RAD;
            rotation_.y = rotation_buffer[1] * (-DEG_2_RAD);
            rotation_.z = (rotation_buffer[0] - 180.0f) * (-DEG_2_RAD);
        } // Otherwise  use old data (so no action is necessary)
        // Since Adafruit BNO055 sensor automatically filters data for Euler vector there is no need to use low pass filter

        // Get current time
        ros::WallTime time_curr = ros::WallTime::now();

        // Read acceleration data and (if read succeeded) scale it
        float accel_buffer[3] = {0.0f, 0.0f, 0.0f};
        if (bno_ptr_->getVector(accel_buffer, VectorMappings::VECTOR_LINEARACCEL)) {
            // For acceleration data assign correct values to correct variables
            // No need to subtract gravity values (sensor already does that when reading linear acceleration)
            accel_.x = accel_buffer[0];
            accel_.y = -accel_buffer[1];
            accel_.z = -accel_buffer[2];
        } // Otherwise use old data (so no action is necessary)

        // Calculate integral
        velocity_ = accel_ * (time_curr - time_prev_).toSec() + velocity_prev_; // ############### use filetered velocity instead of previous?   

        // Low pass filter
        velocity_filtered_ = velocity_filtered_ * 0.63946 + velocity_ * 0.18027 + velocity_prev_ * 0.18027;

        // Update remaining values
        time_prev_ = time_curr;
        velocity_prev_ = velocity_;
    }

    /**
     * @brief Function called automatically by ros::Timer in regular time intervals to publish processed data to /imu_data topic
     * There is no need for user to call this function manually!
     * @param event Struct holding some basic time-oriented data, passed by ros
     */
    void publishSensorData() {
        // Create message and fill it with basic data
        tank_controller::imuData msg;
        msg.header.stamp = ros::Time::now();

        // Fill calibration data
        msg.accelerometerCalib = accel_calib_;
        msg.gyroscopeCalib = gyro_calib_;

        // Fill velocity data
        msg.linear.x = velocity_filtered_.x;
        msg.linear.y = velocity_filtered_.y;
        msg.linear.z = velocity_filtered_.z;

        msg.rotation.x = rotation_.x;
        msg.rotation.y = rotation_.y;
        msg.rotation.z = rotation_.z;

        // Publish message
        imu_data_pub_.publish(msg);
    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        // Reset internal variables (juuust in case)
        resetVariables_();

        // Start up threads
        std::thread reader(&ImuBridge::startReadingData_, this);
        std::thread publisher(&ImuBridge::startPublishingData_, this);

        // Wait for user to close node
        ROS_INFO("Main program loop has been started");
        ros::spin();

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
        // Reset calibration values
        accel_calib_ = 0U;
        gyro_calib_ = 0U;

        // Reset velocity but keep timer running!
        time_prev_ = ros::WallTime::now();
        accel_.setZero();
        velocity_.setZero();
        velocity_prev_.setZero();
        velocity_filtered_.setZero();

        // No need to reset rotation data
    }

    /**
     * @brief Function passed to std::thread. Takes care of reading sensor data
     */
    void startReadingData_() {
        while(ros::ok()) {
            this->readSensorData();
            delay(read_wait_);
        }

        std::cout << "|===== Reader thread has been stopped =====|" << std::endl;
    }

    /**
     * @brief Function passed to std::thread. Takes care of publishing sensor data
     */
    void startPublishingData_() {
        while (ros::ok()) {
            this->publishSensorData();
            this->publish_rate_.sleep();
        }

         std::cout << "|===== Publisher thread has been stopped =====|" << std::endl;
    }

    std::string node_name_;
    
    uint8_t accel_calib_;
    uint8_t gyro_calib_;
    
    Vector accel_;
    ros::WallTime time_prev_;
    Vector velocity_;
    Vector velocity_prev_;
    Vector velocity_filtered_;

    Vector rotation_;

    std::unique_ptr<ConnectionBridge> bno_ptr_;

    ros::NodeHandle nh_;
    int read_wait_;
    ros::Rate publish_rate_;
    ros::ServiceServer reset_vars_ser_;
    ros::Publisher imu_data_pub_;
};


int main() {
    signal(SIGINT, rosSigintHandler);
    ImuBridge imu_bridge = {};
    return imu_bridge.run();
}


//     ConnectionBridge bno = {OperationMode::OPERATION_MODE_IMUPLUS};
//     bno.setExtCrystalUse(true);
//     ros::init(ros::M_string(), "imu_test");
//     float accel_vec[3] = {};
//     float gyro_vec[3] = {};
//     float pos_vec[3] = {};
//     float rot_vec[3] = {};
//     byte sys_stat = 0;
//     byte gyro = 0;
//     byte accel = 0;
//     byte mag = 0;
//     bool fully_calib = false;
//     ros::WallTime prev_time = ros::WallTime::now();

//     while (true) {
//         bno.getVector(accel_vec, VectorMappings::VECTOR_LINEARACCEL);
//         bno.getVector(gyro_vec, VectorMappings::VECTOR_GYROSCOPE);

//         ros::WallTime curr_time = ros::WallTime::now();
//         double t_delta = (curr_time - prev_time).toSec();
//         prev_time = curr_time;

//         pos_vec[0] += 0.5 * accel_vec[0] * t_delta * t_delta;
//         pos_vec[1] += 0.5 * accel_vec[1] * t_delta * t_delta;
//         pos_vec[2] += 0.5 * accel_vec[2] * t_delta * t_delta;

//         rot_vec[0] += 0.5 * gyro_vec[0] * t_delta * t_delta;
//         rot_vec[1] += 0.5 * gyro_vec[1] * t_delta * t_delta;
//         rot_vec[2] += 0.5 * gyro_vec[2] * t_delta * t_delta;
        
//         // bno.getCalibration(sys_stat, gyro, accel, mag);
//         // fully_calib = bno.isFullyCalibrated();
//         std::cout << std::fixed << std::setw(8) << std::setprecision(4) 
//                     << "Accel:\n"
//                     << "  x = " << pos_vec[0] << "\n"
//                     << "  y = " << pos_vec[1] << "\n"
//                     << "  z = " << pos_vec[2] << "\n"
//                     << "Gyro:\n"
//                     << "  x = " << rot_vec[0] << "\n"
//                     << "  y = " << rot_vec[1] << "\n"
//                     << "  z = " << rot_vec[2] << "\n"
//                     << "Time: " <<  t_delta
//                     << std::endl;
//                     // std::cout << "Calib (" << (fully_calib ? "is" : "isn't") << " fully calibrated):\n"
//                     // << "  acc = " << (int) accel << "\n"
//                     // << "  gyr = " << (int) gyro << "\n"
//                     // << "  mag = " << (int) mag << "\n" << std::endl;
//         delay(10);
//     }
// }


#endif