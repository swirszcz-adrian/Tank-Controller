#define MAX_STEERING 9.9f
#define WAIT_TIME 2U

#include "ros/ros.h"
#include "tank_controller/motorsManualControl.h"
#include "tank_controller/motorsAutoControl.h"
#include "tank_controller/motorsData.h"
#include "tank_controller/globalStop.h"
#include "tank_controller/motorsParameters.h"
#include "tank_controller/vector3D.h"
#include "tank_controller/piParams.h"

#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <vector>
#include <iterator>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"


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
 * @brief Simple struct for holding parameters of motor's PIcontroller
 */
struct PIparams {
  PIparams() : action(true), 
               update(false), 
               requested_kp(30.0f),
               requested_ki(0.3f),
               actual_kp(std::nanf("")),
               actual_ki(std::nanf("")) {}
  bool action;        // If value is true get or set PI controller's parameters
  bool update;        // Decide whether to get (false) or set (true) controller's parameters
  float requested_kp;
  float requested_ki;
  float actual_kp;
  float actual_ki;
};


/**
 * @brief Class responsible for creating and running node which takes care of comunicationg with Arduino board.
 */
class ArduinoBridge {
private:
  /**
   * @brief This static function HAS to be called at the beggining of contructor's initializer list, otherwise program will compile, but node won't start.
   * This is because ros::init() has to be called before defining any ros-based field/variable, so for example before ros::NodeHandle nh_ is defined
   * ... and the only way to do it is by calling this function in constructor's initializer list (since constructor's body is executed after creating variables' definitions).
   * Ohhh and you can't simply call ros::init() in initializer list, since it doesn't return anything.
   * @cite https://www.youtube.com/watch?v=0oBx7Jg4m-o&ab_channel=ASIDs
   * @param node_name a string containings node's name
   * @return node_name (the same string you passed as a parameter)
   */
  static std::string rosInit(std::string node_name) {
    // Initialize ROS without any remapping arguments (node is quite simple so they won't be needed)
    ros::init(ros::M_string(), node_name);
    return node_name;
  }


public:
  /**
   * @brief Construct a new ArduinoBridge object, that is object of class responsible for creating and running node, 
   * which takes care of comunicationg with Arduino board.
   * @param port name of USB port to which Arduino board is connected
   * @param baudrate USB port's baudare. Set the same as in the Arduino board
   * @param loop_rate rate at main program loop will run (aka. rate at which messages will be read and send) [Hz]
   */
  ArduinoBridge(std::string serial_port="/dev/ttyACM0", int serial_baudrate=9600, int loop_rate = 30) 
               : node_name_(ArduinoBridge::rosInit("arduino_bridge"))   // I still don't believe I had to this this... thing
               , nh_(ros::NodeHandle())
               , loop_rate_(ros::Rate(loop_rate))
               , requested_global_stop_(true) 
               , actual_global_stop_(true)
               , controller_connected_(false)
               , manual_control_(false)
               , last_manual_msg_(0U)
               , last_auto_msg_(0U)
               , steering_left_(0.0f) 
               , steering_right_(0.0f)
               , velocity_left_(0.0f)
               , velocity_right_(0.0f)
               , pi_left_()
               , pi_right_() {
    ROS_INFO("Starting up %s node:", node_name_.c_str());

    // Create service server, that receives requests from /global_stop topic and passes them to
    // function globalStopServer which, is a method of ArduinoBridge class and belongs to "this" object 
    gl_stop_ser_ = nh_.advertiseService("/global_stop", &ArduinoBridge::globalStopServer, this);
    ROS_INFO(" - /global_stop service created");

    // Create service server, that receives requests from /motors_parameters topic and passes them to
    // function motorsParametersServer which, is a method of ArduinoBridge class and belongs to "this" object
    params_ser_ = nh_.advertiseService("/motors_parameters", &ArduinoBridge::motorsParametersServer, this);
    ROS_INFO(" - /motors_parameters service created");

    // Create subscriber, that receives data from /motors_manual_control topic with queue size equal to 1 and calls back
    // function motorsManualControlCallback, which is a method of ArduinoBridge class and belongs to "this" object 
    man_ctrl_sub_ = nh_.subscribe("/motors_manual_control", 1, &ArduinoBridge::motorsManualControlCallback, this);
    ROS_INFO(" - /motors_manual_control subscriber created");

    // Create subscriber, that receives data from /motors_auto_control topic with queue size equal to 1 and calls back
    // function motorsAutoControlCallback, which is a method of ArduinoBridge class and belongs to "this" object 
    auto_ctrl_sub_ = nh_.subscribe("/motors_auto_control", 1, &ArduinoBridge::motorsAutoControllCallback, this);
    ROS_INFO(" - /motors_auto_control subscriber created");

    // Create publisher that sends messages of type tank_controller/motorsData to /motors_data topic with message queue size equal to 1
    data_pub_ = nh_.advertise<tank_controller::motorsData>("/motors_data", 1);
    ROS_INFO(" - /motors_data publisher created");

    // Initiate serial connection
    ser_.setPort(serial_port);
    ser_.setBaudrate(serial_baudrate);
    ser_.open();
    if (!ser_.isOpen()) { ROS_FATAL(" - failed to open serial port"); }
    else { ser_.flush(); ROS_INFO(" - serial port opened"); }
    
    ROS_INFO("Initialization finished");
  }


  // Remove copy constructor and assignement operator since they won't be needed
  ArduinoBridge(const ArduinoBridge &) = delete;
  ArduinoBridge& operator=(const ArduinoBridge &) = delete;

  /**
   * @brief Destroy the ArduinoBridge object, close serial port and print short message
   */
  ~ArduinoBridge() {
    if (ser_.isOpen()) { ser_.close(); }
    std::cout << "|===== Node has been successfully destroyed =====" << std::endl;
  }

  /**
   * @brief Converts velocity data from global (linear in [m/s] and angular in [rad/s]) to motor-specific ([ticks/ms] for both left and right motor)
   * @param linear equivalent of ROS'es twist.linear.x [m/s]
   * @param angular equivalent of ROS'es twist.angular.z [m/s]
   * @param left_motor reference to variable holding left motor's steering [ticks/ms]
   * @param right_motor reference to variable holding right motor's steering [ticks/ms]
   */
  void convertGlobalToMotors(const float &linear, const float &angular, float &left_motor, float &right_motor) {
    // Robot-specific constants:
    // - wheel radius, r = 0.073 [m]
    // - wheels spacing, d = 0.18 [m]
    // - ticks per rotation, tr = 624

    // Convert linear velocity from [m/s] to [ticks/ms]
    // motors_linear = linear / (2 * pi * r) * tr / 1000
    // where: 1 / (2 * pi * r) * tr / 1000 = 1.360448
    float motors_linear = linear * 1.360448f;

    // Convert angular velocity from [rad/s] to [ticks/ms]
    // motors_angular = angular / (2 * pi) * (pi * d) / (2 * pi * r) * tr / 1000
    // where: 1 / (2 * pi) * (pi * d) / (2 * pi * r) * tr / 1000 = 0.12244
    float motors_angular = angular * 0.12244f;

    // Add velocities, contstrain results and update variables
    left_motor = std::min(MAX_STEERING, std::max(-MAX_STEERING, motors_linear - motors_angular));
    right_motor = std::min(MAX_STEERING, std::max(-MAX_STEERING, motors_linear + motors_angular));
  }

  /**
   * @brief Converts velocity data from motor-specific ([ticks/ms] for both left and right motor) to global (linear in [m/s] and angular in [rad/s])
   * @param left_motor left motor's steering [ticks/ms]
   * @param right_motor right motor's steering [ticks/ms]
   * @param linear reference to variable holding equivalent of ROS'es twist.linear.x [m/s]
   * @param angular reference to variable holding equivalent of ROS'es twist.angular.z [m/s]
   */
  void convertMotorsToGlobal(const float &left_motor, const float &right_motor, float &linear, float &angular) {
    // Robot-specific constants:
    // - wheel radius, r = 0.073 [m]
    // - wheels spacing, d = 0.18 [m]
    // - ticks per rotation, tr = 624
    
    // Extract linear and angular velocity
    float motors_linear = 0.5f * (right_motor + left_motor);
    float motors_angular = motors_linear - left_motor;

    // Convert linear velocity from [ticks/ms] to [m/s]
    // linear = motors_linear * 2 * pi * r * 1000 / tr
    // where: 2 * pi * r * 1000 / tr = 0.735052
    linear = motors_linear * 0.735052f;

    // Convert angular velocity from [ticks/ms] to [rad/s]
    // angular = motors_angular * (2 * pi) / (pi * d) * (2 * pi * r) * 1000 / tr
    // where: (2 * pi) / (pi * d) * (2 * pi * r) * 1000 / tr = 8.167246
    angular = motors_angular * 8.167246f;
  }


  /**
   * @brief Function called automatically whenever new globalStop request is received by service server.
   * There is no need for user to call this function manually!
   * @param req reference to request message, passed automatically by ROS
   * @param res reference to response message, passed automatically by ROS
   * @return true (always)
   */
  bool globalStopServer(tank_controller::globalStop::Request &req, tank_controller::globalStop::Response &res) {
    // Simply update required variable
    requested_global_stop_ = req.stop;
    if (req.stop) {
      ROS_INFO("Received request to ACTIVATE global stop");
      // std::cout << std::boolalpha << pi_left_.action << " " << pi_right_.action << std::endl;
    } else {
      ROS_INFO("Received request to DEACTIVATE global stop");
    }

    return true;
  }


  /**
   * @brief Function called automatically whenever new motorsParameters request is received by service server.
   * There is no need for user to call this function manually!
   * @param req reference to request message, passed automatically by ROS
   * @param res reference to response message, passed automatically by ROS
   * @return true (always)
   */
  bool motorsParametersServer(tank_controller::motorsParameters::Request &req, tank_controller::motorsParameters::Response &res) {
    // Fill header data of response
    res.header.stamp = ros::Time::now();
    
    // If requested to override parameters, check if they are within acceptable range (0 ... 99.9)
    if (req.override_values) {
      if (0.0f <= req.left_motor.kp <= 99.9f && 0.0f <= req.left_motor.ki <= 99.9f && 0.0f <= req.right_motor.kp <= 99.9f && 0.0f <= req.right_motor.ki <= 99.9f) {
        // Update requested parameters
        pi_left_.requested_kp = req.left_motor.kp;
        pi_left_.requested_ki = req.left_motor.ki;
        pi_right_.requested_kp = req.right_motor.kp;
        pi_right_.requested_ki = req.right_motor.ki;

        // Set flag to update controller's values
        pi_left_.update = true;
        pi_right_.update = true;

        // Set flag that controller requires action
        pi_left_.action = true;
        pi_right_.action = true;

        // Fill feadback field
        res.feedback = "New values accepted. Please wait a second while node updates parameters";
        ROS_INFO("Received new PI controllers' parameters");

      // If values are not within range send appropriate feedback string
      } else {
        res.feedback = "Values are outside permitted range (0 ... 99.9). No changes have been made";
        ROS_WARN("Received incorrect PI parameters");
      }
      // In both cases set other response fields to NaN
      res.left_motor.kp = std::nanf("");
      res.left_motor.ki = std::nanf("");
      res.right_motor.kp = std::nanf("");
      res.right_motor.ki = std::nanf("");

    // Reqested only to get current parameters
    } else {
      // Fill current parameters values
      res.left_motor.kp = pi_left_.actual_kp;
      res.left_motor.ki = pi_left_.actual_ki;
      res.right_motor.kp = pi_right_.actual_kp;
      res.right_motor.ki = pi_right_.actual_ki;

      // Update actual parameters
      // Commented out, since the only time parameters can change is after setting new values and this opertion also updated actual parameters
      // pi_left_.action = true;
      // pi_right_.action = true;

      // Fill feedback field
      res.feedback = "Sent current parameters";
      ROS_INFO("Uploading current PI parameters");
    }
    
    return true;
  }


  /**
   * @brief Function called automatically, whenever new manual control data is received.
   * There is no need for user to call this function manually!
   * @param msg Passed automatically by ROS subscriber
   */
  void motorsManualControlCallback(const tank_controller::motorsManualControl::ConstPtr &msg) {
    // Get last time manual control message was received
    last_manual_msg_ = msg->header.stamp.sec;

    // Update control variables
    if (!controller_connected_ && msg->isConnected) {
      controller_connected_ = true;
      ROS_INFO("Controller has been connected");
    } else if (controller_connected_ && !msg->isConnected) {
      controller_connected_ = false;
      ROS_WARN("Controller has been disconnected");
    }
    if (!manual_control_ && msg->takeControl) {
      manual_control_ = true;
      ROS_WARN("Switching to manual control");
    } else if (manual_control_ && !msg->takeControl) {
      manual_control_ = false;
      ROS_WARN("Switching to automatic control");
    }

    // Only update steering if controller is connected and manual control is ON
    if (controller_connected_ && manual_control_) {
      convertGlobalToMotors(msg->linear.x, msg->angular.z, this->steering_left_, this->steering_right_);
    }
  }

  /**
   * @brief Function called automatically, whenever new auto control data is received.
   * There is no need for user to call this function manually!
   * @param msg Passed automatically by ROS subscriber
   */
  void motorsAutoControllCallback(const tank_controller::motorsAutoControl::ConstPtr &msg) {
    // Get last time auto control message was received
    last_auto_msg_ = msg->header.stamp.sec;

    // Update steering if either controller is disconected or manual control is OFF
    if (!controller_connected_ || !manual_control_) {
      convertGlobalToMotors(msg->linear.x, msg->angular.z, this->steering_left_, this->steering_right_);
    }
  }


  /**
   * @brief Function called inside loop to publish actual velocity and global stop status.
   */
  void motorsDataPublish() {
    // Create message and fill in header data
    tank_controller::motorsData msg;
    msg.header.stamp = ros::Time::now();

    // Fill global stop
    msg.isStopped = actual_global_stop_;

    // Conver and fill velocity data
    convertMotorsToGlobal(this->velocity_left_, this->velocity_right_, msg.linear.x, msg.angular.z);

    // Publish message
    data_pub_.publish(msg);
  }


  /**
   * @brief Read data from serial buffer and (if data wasn't corrupted) update appropriate variables.
   * This function has to be called inside a loop.
   * This function can be called in second thread.
   */
  void serialReadOnce() {
    // Read all complete datasets from serial buffer line by line
    while (ser_.available() > 12) {
      // Read serial message
      std::string message = ser_.readline(32U, "\n");

      // Strip message of any trailing whitespaces
      size_t start = message.find_first_not_of(" ");
      size_t end = message.find_last_not_of(" ");
      if (start >= end) { continue; }
      else { message = message.substr(start, end); }
      if (message.length() < 14) { continue; }

      // Separate message into command and value
      std::string command = message.substr(0, 2);
      std::string value = message.substr(3);

      // COMMAND TYPE: velocity data
      // example value: "+1.42 -0.66"
      if ((command == "MV" || command == "ST") && value.length() > 9) {
        // Get actual global stop
        actual_global_stop_ = (command == "ST");

        // Try to get float values out of string
        try {
          float left = std::stof(value.substr(0, 5));
          float right = std::stof(value.substr(6, 11));

          velocity_left_ = left;
          velocity_right_ = right;
          
        // If it fails simply continue loop
        } catch (...) {   // catching every exception like this isn't good design, but I really want to catch all the exceptions
          continue;
        }

      // COMMAND TYPE: get PI controllers' parameters 
      // example value: "30.00 00.30"
      } else if ((command == "PL" || command == "PR") && value.length() > 9) {
        // Get pointer to the appropriate struct
        // Instead of this you could use if..else inside try..catch clauzule
        PIparams *PIcontroller = &(command == "PL" ? this->pi_left_ : this->pi_right_);
        
        // Try to get float values out of string
        try {
          float kp = std::stof(value.substr(0, 5));
          float ki = std::stof(value.substr(6, 11));

          // Update parameters held in Raspberry Pi
          PIcontroller->actual_kp = kp;
          PIcontroller->actual_ki = ki;

          // If Arduino had to override its Pi parameters check if it has been done correctly
          // (aka. check if actual and requested values are similiar)
          if (PIcontroller->update) {
            if (std::fabs(PIcontroller->requested_kp - PIcontroller->actual_kp) < 0.02f && std::fabs(PIcontroller->requested_ki - PIcontroller->actual_ki) < 0.02f) {
              PIcontroller->update = false;
              PIcontroller->action = false;
            }

          // If request was only to get parameter's values change "action" variable to false
          } else {
            PIcontroller->action = false;
          }

        // In case of failure continue loop
        } catch(...) {
          continue;
        }
      }
    }  
  }


  /**
   * @brief Convert variables into string readable by Arduino board, then send it via serial.
   * This function has to be called inside a loop.
   * This function can be called in second thread.
   */
  void serialWriteOnce() {
    // GLOBAL STOP SECTION
    if (requested_global_stop_) {
      ser_.write("ST ON\n");
    } else if (actual_global_stop_) {
      ser_.write("ST OFF\n");
    } else {
    
    // STEERING SECTION
      // Create temporary string streams
      std::stringstream left_stream;
      std::stringstream right_stream;

      // Write strings to buffers with calculated precisions and optional "+" sign
      left_stream << (steering_left_ >= 0.0f ? "+" : "") << std::fixed <<std::setprecision(2) << steering_left_;
      right_stream << (steering_right_ >= 0.0f ? "+" : "") << std::fixed << std::setprecision(2) << steering_right_;

      // Write complete data to serial
      ser_.write("MV " + left_stream.str() + " " + right_stream.str() + "\n");
    }

    // PICONTROLLER PARAMETERS SECTION
    // One big else-if clauzure is used here instead of two if clauzures to prevent filling Arduino's serial buffer with a load of messages in a single loop
    if (pi_left_.action) {
      // Override left PI controller's parameters
      if (pi_left_.update) {
        // The process here is similiar to steering but, instead of adding "+" sign, additional 0 will be added if number is smaller than 10
        std::stringstream kp_stream;
        std::stringstream ki_stream;

        kp_stream << (pi_left_.requested_kp < 10.0f ? "0" : "") << std::fixed << std::setprecision(2) << pi_left_.requested_kp;
        ki_stream << (pi_left_.requested_ki < 10.0f ? "0" : "") << std::fixed << std::setprecision(2) << pi_left_.requested_ki;

        ser_.write("PL " + kp_stream.str() + " " + ki_stream.str() + "\n");

      // Get left PI controller's parameters
      } else {
        ser_.write("PL GET\n");
      }
    } else if (pi_right_.action) {
      // Override right PI controller's parameters
      if (pi_right_.update) {
        std::stringstream kp_stream;
        std::stringstream ki_stream;

        kp_stream << (pi_right_.requested_kp < 10.0f ? "0" : "") << std::fixed << std::setprecision(2) << pi_right_.requested_kp;
        ki_stream << (pi_right_.requested_ki < 10.0f ? "0" : "") << std::fixed << std::setprecision(2) << pi_right_.requested_ki;

        ser_.write("PR " + kp_stream.str() + " " + ki_stream.str() + "\n");

      // Get right PI controller's parameters
      } else {
        ser_.write("PR GET\n");
      }
    }
  }


  /**
   * @brief Main program loop. Call it to run nodes and entire program logic.
   * @return EXIT_SUCCESS if closed properly
   */
  int run() {
    // Wait 3 seconds, just to be sure
    ROS_INFO("Starting up main program loop...");
    #ifdef _WIN32
      Sleep(3000);         // 3 s
    #else
      usleep(3000 * 1000); // 3 s
    #endif

    // Check if serial port is still open. If not close program
    if (!ser_.isOpen()) { ROS_FATAL("Serial port could not open, closing main loop"); return EXIT_FAILURE; }
    ser_.flush();
    ROS_INFO("Main program loop has been started");

    // Check if node should run (aka. if terminal hasn't been closed, Ctrl+C wan't passed etc.)
    while(ros::ok()) {
      // Read serial data
      serialReadOnce();

      // Publish motors' data
      motorsDataPublish();

      // Check if subscribers received messages
      ros::spinOnce();

      // Write serial data
      serialWriteOnce();

      // if robot is not stopped, check how long ago any kind of message was received
      if (!requested_global_stop_) {
        uint32_t current_time = ros::Time::now().sec;

        // If no new message was received for over WAIT_TIME seconds, assume connection was lost
        if ((manual_control_ && current_time - last_manual_msg_ > WAIT_TIME) || (!manual_control_ && current_time - last_auto_msg_ > WAIT_TIME)) {
          requested_global_stop_ = true;
          controller_connected_ = false;
          manual_control_ = false;
          ROS_WARN("No new message has been received for over %u seconds. Activating global stop", (uint32_t) WAIT_TIME);
        }
      }

      // Wait remaining loop time
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
    return this->run(); // "this" pointer is unnecessary but it looks nice so it stays
  }  


private:
  std::string node_name_;

  bool requested_global_stop_;
  bool actual_global_stop_;
  bool controller_connected_;
  bool manual_control_;
  
  uint32_t last_manual_msg_;
  uint32_t last_auto_msg_;
  
  float steering_left_;
  float steering_right_;
  float velocity_left_;
  float velocity_right_;

  PIparams pi_left_;
  PIparams pi_right_;

  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  ros::ServiceServer gl_stop_ser_;
  ros::ServiceServer params_ser_;
  ros::Subscriber man_ctrl_sub_;
  ros::Subscriber auto_ctrl_sub_;
  ros::Publisher data_pub_;
  
  serial::Serial ser_;
};


int main(int argc, char **argv) {
  ArduinoBridge arduino_bridge = {};
  return arduino_bridge.run();
}