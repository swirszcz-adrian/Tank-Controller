#include "ros/ros.h"
#include "serial/serial.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <iterator>
#include <unistd.h>


/**
 * @brief Simple function for splitting string based on separator
 * @param str string to split
 * @param sep separator
 * @return std::vector<std::string> vector of separated strings
 */
std::vector<std::string> split(const std::string &str, char sep=' ') {
  // Copy string's contents into std::istringstream object
  std::istringstream iss(str);
  
  // Create two variables:
  // one beeing a temporary buffer for the std::getline() function (as it can only write to previously created buffer strings) 
  // second one to store splitted strings
  std::string temp;
  std::vector<std::string> result;
  
  // Create std::back_insert_iterator to write data from temporary buffer into result vector
  // Note: using push_back() method instead of iterator caused fragmentation errors
  std::back_insert_iterator<std::vector<std::string>> ptr = std::back_inserter(result);
  
  // While std::istringstream object is not empty get new strings and add them to the result vector using std::back_insert_iterator
  while (std::getline(iss, temp, sep)) {
      *ptr++ = temp;
  }

  return result;
}


/**
 * @brief Function used to test serial port quality
 * @param nh node_handle
 * @param loop_rate rate at which sendmessages and check serial buffer
 */
int testSerialQuality(ros::NodeHandle nh, ros::Rate loop_rate) {
    ROS_INFO("Attempting to open serial port");

    // Clear and open csv file
    std::ofstream my_file;
    my_file.open("/home/swirszcz/Desktop/testing_data/connection_data.csv", std::ofstream::out | std::ofstream::trunc);
    my_file << "time,rpi_good_msgs,rpi_bad_msgs,ard_good_msgs,ard_bad_msgs,\n";

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

    // Ask Arduino to clear message buffer and turn off global stop
    ser.write("CLEAR DATA\n");
    usleep(33333);
    ser.write("ST OFF\n");
    usleep(33333);
    ser.write("MV +2.00 +2.00\n");
    usleep(33333);

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
                    ard_good_msgs = std::stoi(values.substr(0, 5));
                    ard_bad_msgs = std::stoi(values.substr(6, 11));
                    received_good_msg = true;

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
    ser.close();
    my_file.close();
    return EXIT_SUCCESS;
}


int main() {
    // Initialize ros, create node handle and 30 Hz rate
    ros::init(ros::M_string(), "tester");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    return testSerialQuality(nh, loop_rate);
}