// ##### WARNING #####
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
#include "tank_controller/motorsData.h"
#include "tank_controller/motorsManualControl.h"

#include <wiringPi.h>
#include <lcd.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <array>
#include <iterator>
#include <algorithm>


//USE WIRINGPI PIN NUMBERS
#define LCD_RS  25               //Register select pin
#define LCD_E   24               //Enable Pin
#define LCD_D4  23               //Data pin 4
#define LCD_D5  22               //Data pin 5
#define LCD_D6  21               //Data pin 6
#define LCD_D7  14               //Data pin 7

/**
 * @brief Function used to write messages to terminal and retrieve back output
 * @param cmd command passed to terminal
 * @return std::string output returned from terminal
 */
std::string terminalCommand(const std::string cmd) {
    // Create buffers to which output  will be written into
    char buffer[128];
    std::string result = "";

    // Execute command using popen() function
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) { 
        throw std::runtime_error("popen() failed!"); 
    }
    
    // Move data from char array to  std::string
    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw std::runtime_error("Failed to copy data!");
    }

    // Close pipe, return result 
    pclose(pipe);
    return result;
}


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
 * @brief Simple enum for choosing between lcd's top or bottom row 
 */
enum LcdRows {
    TOP_ROW = 0,
    BOTTOM_ROW = 1
};


/**
 * @brief Simple enum for choosing between printing in left or right half of a row
 */
enum LcdHalves {
    LEFT_HALF = 0,
    RIGHT_HALF = 8,
};


// List of all nodes
const std::array<std::string, 3> node_list = {
    "/arduino_bridge",
    "/gamepad_bridge",
    "/imu_bridge"
};


/**
 * @brief Class responsible for displaying data on 16x2 LCD screen
 */
class DisplayControl {
public:
    /**
     * @brief Construct a new DisplayControl object responsible for displaying data on 16x2 LCD screen
     */
    DisplayControl() {
        // Initiate wiringPi
        wiringPiSetup();

        // Initiate lcd
        lcd_ = lcdInit (2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);

        // Clear lcd screen
        lcdClear(lcd_);

        // Wait for quater of second for screen to clear
        usleep(250000);
    }

    // Remove copy constructor and assignement operator since they won't be needed
    DisplayControl(const DisplayControl &) = delete;
    DisplayControl& operator=(const DisplayControl &) = delete;

    //Default move constructor
    DisplayControl(DisplayControl &&) = default;

    /**
     * @brief Destroy the DisplayControl object and clear display
     */
    ~DisplayControl() {
        lcdClear(lcd_);
    }

    /**
     * @brief Prints given message in given row
     * @param row whether to print message in top or bottom row
     * @param msg text of message
     * @return true if print succeeded
     * @return false if print failed
     */
    bool printRow(LcdRows row, std::string msg) {
        if (msg.length() <= 16) {
            // Select row
            lcdPosition(lcd_, 0, row);

            // Print message (append is used to clear remaining space in row)
            lcdPuts(lcd_, msg.append(16 - msg.length(), ' ').c_str());
            
            return true;
        }

        return false;
    }

    /**
     * @brief Prints given message in given half of a given row
     * @param row whether to print message in top or bottom row
     * @param half whether to print message in left or right half of a row
     * @param msg text of message
     * @return true if print succeeded
     * @return false if print failed
     */
    bool printQuarter(LcdRows row, LcdHalves half, std::string msg) {
        if (msg.length() <= 8) {
            lcdPosition(lcd_, half, row);
            lcdPuts(lcd_, msg.append(8 - msg.length(), ' ').c_str());

            return true;
        }

        return false;
    }

    /**
     * @brief Clear lcd screen
     */
    void clearScreen() {
        lcdClear(lcd_);
    }
    

private:
    int lcd_;
};


/**
 * @brief Class responsible for getting data from ros topics ans parsing it into lcd display
 */
class DisplayBridge {
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
     * @brief Construct a new DisplayBridge object, responsible for getting data from ros topic and parsing it into lcd display
     * @param display reference to DisplayControl object, used to directly control lcd
     * @param loop_rate rate at which new data will be aquired (no need to make it high, since lcd screen takes a while to update)
     */
    DisplayBridge(DisplayControl &display, int loop_rate = 2)
                 : node_name_(DisplayBridge::rosInit("display_bridge")) 
                 , display_(std::move(display))
                 , nh_(ros::NodeHandle())
                 , loop_rate_(loop_rate) {
        ROS_INFO("Starting up %s node:", node_name_.c_str());

        // Create subscriber, that receives data from /motors_manual_control topic with queue size equal to 1 and calls back
        // function motorsManualControlCallback, which is a method of DisplayBridge class and belongs to "this" object 
        man_ctrl_sub_ = nh_.subscribe("/motors_manual_control", 1, &DisplayBridge::motorsManualControlCallback, this);
        ROS_INFO(" - /motors_manual_control subscriber created");

        // Create subscriber, that receives data from /motors_data topic with queue size equal to 1 and calls back
        // function motorsDataCallback, which is a method of DisplayBridge class and belongs to "this" object 
        mot_data_sub_ = nh_.subscribe("/motors_data", 1, &DisplayBridge::motorsDataCallback, this);
        ROS_INFO(" - /motors_data subscriber created");

        ROS_INFO("Initialization finished");
    }

    // Remove copy constructor and assignement operator since they won't be needed
    DisplayBridge(const DisplayBridge &) = delete;
    DisplayBridge& operator=(const DisplayBridge &) = delete;

    /**
     * @brief Destroy the DisplayBridge object, print message
     */
    ~DisplayBridge() {
        std::cout << "|===== Node has been successfully destroyed =====" << std::endl;
    }

    /**
     * @brief Function called automatically by ros
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ros subscriber
     */
    void motorsManualControlCallback(const tank_controller::motorsManualControl::ConstPtr &msg) {
        if (msg->isConnected) {
            if (msg->takeControl) {
                // Gamepad connected and taking control
                display_.printQuarter(TOP_ROW, RIGHT_HALF, "BT: CTRL");

            } else {
                // Gamepad connected  but not taking control
                display_.printQuarter(TOP_ROW, RIGHT_HALF, "BT: STBY");
            }
        } else {
            // Gamepad not connected
            display_.printQuarter(TOP_ROW, RIGHT_HALF, "BT: OFF");
        }
    }

    /**
     * @brief Function called automatically by ros
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ros subscriber
     */
    void motorsDataCallback(const tank_controller::motorsData::ConstPtr &msg) {
        if (msg->isStopped) {
            display_.printQuarter(TOP_ROW, LEFT_HALF, "STOPPED");
        } else {
            display_.printQuarter(TOP_ROW, LEFT_HALF, "MOVING");
        }
    }


    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        // Before continuing clear screen and print default messages
        display_.clearScreen();
        display_.printQuarter(TOP_ROW, LEFT_HALF, "STOPPED");
        display_.printQuarter(TOP_ROW, RIGHT_HALF, "BT: OFF");


        ROS_INFO("Main program loop has been started");

        // Use loop to keep low update rate
        while(ros::ok()) {
            ros::spinOnce();
            loop_rate_.sleep();
        }
        
        // Loop finished == Destroy node
        ROS_INFO("Main program loop ended, destroying node");
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
    std::string node_name_;
    DisplayControl display_;

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::Subscriber man_ctrl_sub_;
    ros::Subscriber mot_data_sub_;
};


int main() {
    // Start up display
    DisplayControl display = {};

    // Wait for nodes to boot up
    std::cout << "Waiting for other nodes. This may take a while..." << std::endl;
    display.printRow(TOP_ROW, "Wait for nodes..");
    while (true) {
        // Get output of "rosnode list" command
        std::string output = terminalCommand("rosnode list");

        // If output is empty then roscore hasn't been initialized
        if (output == "") {
            display.printRow(BOTTOM_ROW, "Master offline!");
        } else {
            // Otherwise split output into a vector and check whether it contains names of all required nodes
            std::vector<std::string> active_nodes = split(output, '\n');
            int num_online = 0;
            for (auto node : node_list) {
                if (std::find(active_nodes.begin(), active_nodes.end(), node) != active_nodes.end()) { 
                    num_online++; 
                }
            }

            // Display number of active nodes
            display.printRow(BOTTOM_ROW, std::to_string(num_online) + "/" + std::to_string(node_list.size()) + " online");
        
            // If all nodes are online exit loop
            if (num_online == node_list.size()) { break; }
        }

        // Wait a bit before repeating procedure
        usleep(500000);
    }

    // All other nodes are online -> time to start up ros subscriptions
    std::cout << "All other nodes online, starting up data collection" << std::endl;
    DisplayBridge display_bridge = {display};
    display_bridge.run();
}


#endif