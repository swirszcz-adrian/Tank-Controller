#include "ros/ros.h"
#include "tank_controller/i2c_bno055.h"
#include "serial/serial.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <iterator>
#include <unistd.h>
#include <chrono>

using namespace bno055;

/**
 * @brief Function used to test serial port quality
 */
int testSerialConnection() {
    // Initialize ros, create node handle and 30 Hz rate
    ros::init(ros::M_string(), "tester");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    // Clear and open csv file
    std::ofstream my_file;
    my_file.open("/home/swirszcz/Desktop/testing_data/connection_data.csv", std::ofstream::out | std::ofstream::trunc);
    my_file << "time,rpi_good_msgs,rpi_bad_msgs,ard_good_msgs,ard_bad_msgs,\n";

    ROS_INFO("Attempting to open serial port");

    // Initiate serial connection
    serial::Serial ser;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(9600);
    ser.open();

    // Wait for connection
    while (!ser.isOpen()) {
        usleep(500000);
        ser.open();
    }

    // Wait for 3 more seconds so that connection can stabilize
    usleep(3000000);

    ROS_INFO("Port has been opened, collecting data");

    // Ask Arduino to turn off global stop
    ser.write("ST OFF\n");
    usleep(33333);
    ser.write("MV +2.00 +2.00\n");
    usleep(1000000);

    // Get current time
    ros::WallTime start_time = ros::WallTime::now();

    // Write number of good and bad messages
    unsigned int rpi_good_msgs = 0U;
    unsigned int rpi_bad_msgs = 0U;

    // Write arduinos good and bad
    unsigned int ard_good_msgs = 0U;
    unsigned int ard_bad_msgs = 0U;

    // Start sending controls
    while (ros::ok()) {
        // Send steering
        ser.write("MV +2.00 +2.00\n");

        // Get time
        ros::WallTime curr_time = ros::WallTime::now();
        float time = (curr_time - start_time).toSec();

        // Read values
        while (ser.available() > 14) {
            // Whether to clasify message as good or bad
            bool received_good_msg = false;

            // Read serial message
            std::string message = ser.readline(32U, "\n");

            // Strip message of any trailing whitespaces
            size_t start = message.find_first_not_of(" ");
            size_t end = message.find_last_not_of(" ");
            if (start >= end) { rpi_bad_msgs++; continue; }
            else { message = message.substr(start, end); }
            if (message.length() < 14) { rpi_bad_msgs++; continue; }

            // Separate message into command and value
            std::string command = message.substr(0, 2);
            std::string values = message.substr(3);

            // example value: "18000 00001"
            if ((command == "MV" || command == "ST") && values.length() > 9) {
                // Try to get integer values out of string
                try {
                    int new_ard_good_msgs = std::stoi(values.substr(0, 5));
                    int new_ard_bad_msgs = std::stoi(values.substr(6, 11));
                    
                    if (new_ard_good_msgs >= ard_good_msgs && new_ard_bad_msgs >= ard_bad_msgs) {
                        received_good_msg = true;
                        ard_good_msgs = new_ard_good_msgs;
                        ard_bad_msgs = new_ard_bad_msgs;
                    }

                // If it fails mark message as bad
                } catch (...) {   // catching every exception like this isn't good design, but I really want to catch all the exceptions
                    
                }
            } 

            // Update message values
            if (received_good_msg) {
                rpi_good_msgs++;
            } else {
                rpi_bad_msgs++;
                std::cout << message << std::endl;
            }

            // Write data to file
            my_file << time<< "," << rpi_good_msgs << "," << rpi_bad_msgs << "," << ard_good_msgs << "," << ard_bad_msgs << ",\n";
        }

        // If runtime > 60 sec, exit
        if (time > 60.0) { break; }

        // Wait for the remaining loop time
        loop_rate.sleep();
    }

    // Close serial port and save changes to file
    ROS_INFO("Loop ended");
    ser.write("ST ON\n");
    usleep(1000000);
    ser.close();
    my_file.close();
    return EXIT_SUCCESS;
}

/**
 * @brief Function used to get data from IMU and write it to csv file
 */
int writeImuData() {
    // Initialize ros, create node handle
    ros::init(ros::M_string(), "tester");
    ros::NodeHandle nh;
    ROS_INFO("Starting program");

    // Create connection
    ConnectionBridge bno = {OperationMode::OPERATION_MODE_IMUPLUS};
    bno.setExtCrystalUse(true);

    // Open or create csv file
    std::ofstream my_file;
    my_file.open("/home/swirszcz/Desktop/testing_data/imu_data.csv", std::ofstream::out | std::ofstream::trunc);
    my_file << "time,rotation,acceleration,\n";

    // Get current time
    auto start = std::chrono::high_resolution_clock::now();

    ROS_INFO("Starting main loop");
    
    while (ros::ok()) {
        // Get time since start
        auto time_temp = std::chrono::high_resolution_clock::now() - start;
        auto time = std::chrono::duration<double>(time_temp).count();

        // Get IMU data
        float rotation_buffer[3] = {0.0f, 0.0f, 0.0f};
        float accel_buffer[3] = {0.0f, 0.0f, 0.0f};
        if (!bno.getVector(rotation_buffer, VectorMappings::VECTOR_EULER)) { continue; }
        if (!bno.getVector(accel_buffer, VectorMappings::VECTOR_LINEARACCEL)) { continue; }

        // Write scaled data to file
        my_file << std::fixed << std::setprecision(8) << time << "," << -rotation_buffer[0] << "," << accel_buffer[0] << ",\n";

        // If time is greater than 10s -> exit
        if (time > 10) { break; }
        
        // Sleep so that imu can keep up
        delay(10);
    }

    ROS_INFO("Main loop ended");

    // close file
    my_file.close();
    return EXIT_SUCCESS;
}


enum class Steering {
    Triangular,
    Square,
    Sine
};

/**
 * @brief Simple function used to send given steering to Arduino and save output to csv file
 * @param steering type of function that will be used to steer motor
 */
int writeMotorsData(Steering steering) {
    // Initialize ros, create node handle
    ros::init(ros::M_string(), "tester");
    ros::NodeHandle nh;
    
    // Clear and open csv file
    std::ofstream my_file;
    my_file.open("/home/swirszcz/Desktop/testing_data/motors_data.csv", std::ofstream::out | std::ofstream::trunc);
    my_file << "t,w(t),u(t),y(t)\n";

    ROS_INFO("Attempting to open serial port");

    // Initiate serial connection
    serial::Serial ser;
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(9600);
    ser.setTimeout(0, 17, 0, 0, 0);
    ser.open();

    // Wait for connection
    while (!ser.isOpen()) {
        usleep(500000);
        ser.open();
    }

    // Wait for 3 more seconds so that connection can stabilize
    usleep(3000000);

    // Ask Arduino to turn off global stop
    ser.write("ST OFF\n");
    usleep(33333);
    ser.write("MV +0.00 +0.00\n");
    usleep(33333);

    ROS_INFO("Port has been opened, collecting data");

    // Get starting time
    auto start = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {
        // Get time since start
        auto time_temp = std::chrono::high_resolution_clock::now() - start;
        auto time = std::chrono::duration<double>(time_temp).count();

        // Calculate steering
        double ster_value = 0.0;
        switch (steering) {
            case Steering::Triangular:
                ster_value = abs(std::fmod(time, 4.0) - 2.0);
                break;

            case Steering::Square:
                ster_value = std::fmod(time, 4.0) < 2.0 ? 2.0 : 0.0;
                break;

            case Steering::Sine:
                ster_value = 2.0 * std::sin(M_PI_4 * time);
                break;
        }

        // Send steering
        std::stringstream stream;
        stream << (ster_value >= 0.0f ? "+" : "") << std::fixed <<std::setprecision(2) << ster_value;
        ser.write("MV " + stream.str() + " " + stream.str() + "\n");

        // Get data
        while (ser.available() > 14) {
            std::string message = ser.readline(32U, "\n");

            // Strip message of any trailing whitespaces
            size_t start = message.find_first_not_of(" ");
            size_t end = message.find_last_not_of(" ");
            if (start >= end) { continue; }
            else { message = message.substr(start, end); }
            if (message.length() < 14) { continue; }

            // Write data to file
            my_file << std::fixed << std::setprecision(4) << time <<","
                    << message << "\n";
        }

        // If time is greater than 12s -> exit
        if (time > 12.0) { break; }
    }

    // Close serial port and save changes to file
    ROS_INFO("Loop ended");
    ser.write("ST ON\n");
    ser.close();
    my_file.close();
    return EXIT_SUCCESS;
}


int main() {
    // return testSerialConnection();
    return writeImuData();
    // return writeMotorsData(Steering::Sine);
}