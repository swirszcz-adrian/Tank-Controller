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
            lcdPosition(lcd_, 0, row);
            lcdPuts(lcd_, msg.append(16 - msg.length(), ' ').c_str());
            
            return true;
        }

        return false;
    }

    /**
 /**
     * @brief Prints given message in given row and column
     * @param row whether to print message in top or bottom row
     * @param col column from which message will be written (range 0..15)
     * @param msg text of message
     * @return true if print succeeded
     * @return false if print failed
     */
    bool printPosition(LcdRows row, int col, std::string msg) {
        if (0 <= col <= 15 && msg.length() <= 16 - col) {
            lcdPosition(lcd_, col, row);
            lcdPuts(lcd_, msg.c_str());

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

        // Check for updates once a second
        sleep(1);
    }

    display.clearScreen();
}


#endif