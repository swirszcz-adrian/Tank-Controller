#include "ros/ros.h"
#include "tank_controller/motorsManualControl.h"
#include "tank_controller/globalStop.h"

#include <SFML/Graphics.hpp>
#include <math.h>
#include <iostream>


// Define keybindings representing controller's binary inputs
enum keyBindings {
    CROSS = 0,
    CIRCLE = 1,
    TRIANGLE = 2,
    SQUARE = 3,
    L1 = 4,
    R1 = 5,
    L2 = 6,
    R2 = 7,
    SHARE = 8,
    OPTIONS = 9,
    PS_BUTTON = 10,
    L3 = 11,
    R3 = 12
};


// Define keybindings representing controller's analog inputs
// All analog values are float32
#define LT_HORIZONTAL sf::Joystick::Axis::X 
#define LT_VERTICAL sf::Joystick::Axis::Y
#define L2_ANALOG sf::Joystick::Axis::Z             // L2 has both analog and binary input
#define R2_ANALOG sf::Joystick::Axis::R             // R2 has both analog and binary input
#define RT_HORIZONTAL sf::Joystick::Axis::U
#define RT_VERTICAL sf::Joystick::Axis::V
#define ARROWS_HORIZONTAL sf::Joystick::Axis::PovX  // Even though arrows are binary buttons, they are listed as analog input
#define ARROWS_VERTICAL sf::Joystick::Axis::PovY    // Even though arrows are binary buttons, they are listed as analog input


// Analog values mapping (for joysticks and arrows)
/*        
 *        -100
 *          ⬆
 *  -100 ⬅  0  ➡  +100
 *          ⬇
 *         +100
 */
// L2 and R2 values are -100 when not pressed and +100 when fully pressed

/**
 * @brief Class responsible for creating and running node that captures gamepad controller's data and parses it into /motors_manual_control topic.
 */
class GamepadBridge{
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
     * @brief Construct a new GamepadBridge object, which is responsible for creating and running node that captures gamepad controller's data and parses it into /motors_manual_control topic.
     * @param linear_scale max linear velocity [m/s]
     * @param angular_scale max angular velocity (turn rate) [rad/s] 
     * @param deadzone value, under which steering will be rounded to 0 [in range 0 to 1]
     * @param linux_restart_bluetooth_on_node_destroy only useful on linux machines if you want to disconnect gamepad after node closes
     * @param loop_rate rate at which program should read inputs and parse them to ROS topic [Hz]
     */
    GamepadBridge(float linear_scale = 1.5f, float angular_scale = 2.0f * M_PI, float deadzone = 0.1f,  bool linux_restart_bluetooth_on_node_destroy = true, int loop_rate = 30) 
                 : node_name_(GamepadBridge::rosInit("gamepad_bridge"))
                 , nh_(ros::NodeHandle())
                 , loop_rate_(loop_rate)
                 , lin_scale_(linear_scale)
                 , ang_scale_(angular_scale)
                 , controller_connected_(false)
                 , manual_control_(false)
                 , cross_pressed_(false)
                 , restart_bluetooth_(linux_restart_bluetooth_on_node_destroy) {
    ROS_INFO("Starting up %s node:", node_name_.c_str());
    
    // Set deadzone parameter
    deadzone_ = (0.0 <= deadzone < 1 ? deadzone : 0.1f);

    // Create service client, which sends requests of tank_controller::globalStop type and sends them
    // to /global_stop topic
    gl_stop_cln_ = nh_.serviceClient<tank_controller::globalStop>("/global_stop");
    ROS_INFO(" - /global_stop client created");

    // Create publisher that sends messages of type tank_controller/motorsManualControl 
    // to /motors_manual_control topic with message queue size equal to 1
    man_ctrl_pub_ = nh_.advertise<tank_controller::motorsManualControl>("/motors_manual_control", 1);
    ROS_INFO(" - /motors_manual_control publisher created");

    ROS_INFO("Initialization finished");
    }


    // Remove copy constructor and assignement operator since they won't be needed
    GamepadBridge(const GamepadBridge &) = delete;
    GamepadBridge& operator=(const GamepadBridge &) = delete;


    /**
    * @brief Destroy the GamepadBridge object, if needed activate global stop (optionaly restart bluetooth) and print short message.
    */
    ~GamepadBridge() {
        #ifndef _WIN32
        if (restart_bluetooth_) { system("sudo systemctl restart bluetooth"); }
        #endif

        std::cout << "|===== Node has been successfully destroyed =====" << std::endl;
    }

    /**
     * @brief Function used to call global stop service server
     * @param stop activate or deactivate global stop
     */
    void globalStopClient(bool stop) {
        tank_controller::globalStop srv;
        srv.request.header.stamp = ros::Time::now();
        srv.request.stop = stop;
        if (gl_stop_cln_.call(srv)) {
            std::string called_stop = (stop ? "ACTIVATE" : "DEACTIVATE");
            ROS_INFO("Sent request to %s global stop", called_stop.c_str());
        } else {
            ROS_WARN("Failed to contact service server");
        }
    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        ROS_INFO("Attempting to establish connection with controller...");

        // Check if node should run (aka. if terminal hasn't been closed, Ctrl+C wan't passed etc.)
        while(ros::ok()) {
            // Update controller input
            sf::Joystick::update();

            // Check if joystic has been connected/disconnected
            if (sf::Joystick::isConnected(0)) {
                if (!controller_connected_) {
                    controller_connected_ = true;
                    ROS_INFO("Controller has been connected");
                }
            } else {
                // if joystick has been disconnected, make sure to send appropriate message
                if (controller_connected_) {
                    controller_connected_ = false;
                    // Additionally if robot was in manual control when controller disconnected, turn on global stop
                    if (manual_control_) { 
                        manual_control_ = false; 
                        globalStopClient(true);
                    }
                    
                    ROS_WARN("Controller has been disconnected!");
                }
            }

            // Create message, fill header data and isConnecte and takeControl fields
            tank_controller::motorsManualControl msg;
            msg.header.stamp = ros::Time::now();
            msg.isConnected = controller_connected_;
            msg.takeControl = manual_control_;

            // If controller is connected read inputs
            if (controller_connected_) {

                // Triangle is responsible for taking/returning control
                if (sf::Joystick::isButtonPressed(0, keyBindings::TRIANGLE)) {
                    // If L2 and R2 are pressed return control
                    if (sf::Joystick::isButtonPressed(0, keyBindings::L2) && sf::Joystick::isButtonPressed(0, keyBindings::R2)) {
                        // Return control and switch on global stop
                        if (manual_control_) {
                            manual_control_ = false;
                            ROS_WARN("Switching OFF manual controll");
                            globalStopClient(true);
                        } 
                        
                    // Otherwise take control
                    } else {
                        if (!manual_control_) {
                            manual_control_ = true;
                            ROS_WARN("Switching ON manual control");
                            globalStopClient(true);
                        }
                    }
                }

                // Cross is responsible for activating/deactivating global stop
                if (sf::Joystick::isButtonPressed(0, keyBindings::CROSS)) {
                    // Only activate globalstop if cross was pressed in this loop
                    if (!cross_pressed_) {
                        // If L2 and R2 are pressed deactivate global stop
                        if (sf::Joystick::isButtonPressed(0, keyBindings::L2) && sf::Joystick::isButtonPressed(0, keyBindings::R2)) {
                            globalStopClient(false);
                        
                        // Otherwise activate global stop
                        } else {
                            globalStopClient(true);
                        }
                    }

                    cross_pressed_ = true;
                } else {
                    cross_pressed_ = false;
                }

                // Update steering (accounting for deadzone)
                float steering_linear = -0.01f * sf::Joystick::getAxisPosition(0, LT_VERTICAL);
                float steering_angular = -0.01 * sf::Joystick::getAxisPosition(0, LT_HORIZONTAL);

                msg.linear.x = (std::fabs(steering_linear) < deadzone_ ? 0.0f : lin_scale_ * steering_linear);
                msg.angular.z = (std::fabs(steering_angular) < deadzone_ ? 0.0f : ang_scale_ * steering_angular);

            // Otherwise set control values to 0
            } else {
                msg.linear.x = 0.0f;
                msg.angular.z = 0.0f;
            }

            // Publish message
            man_ctrl_pub_.publish(msg);

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
        return this->run();
    }

private:
    std::string node_name_;

    float lin_scale_;
    float ang_scale_;
    float deadzone_;
    
    bool restart_bluetooth_;
    bool controller_connected_;
    bool manual_control_;
    bool cross_pressed_;

    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::ServiceClient gl_stop_cln_;
    ros::Publisher man_ctrl_pub_;
};

int main() {
    GamepadBridge gamepad_bridge = {};
    return gamepad_bridge.run();
}