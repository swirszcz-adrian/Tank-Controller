#include "ros/ros.h"
#include "tank_controller/imuData.h"
#include "tank_controller/motorsData.h"
#include "tank_controller/motorsAutoControl.h"
#include "tank_controller/addRelativeTarget.h"
#include "tank_controller/clearTargets.h"

#include <iostream>
#include <sstream>
#include <math.h>
#include <vector>


/**
 * @brief Simple struct to allow target's data to be stored in std::vector
 */
struct target {
    /**
     * @brief Construct a new target struct with from data
     */
    target(float x, float z, float x_err, float z_err)
          : x(x)
          , z(z)
          , x_err(x_err)
          , z_err(z_err) {}

    /**
     * @brief Return stringstream representation of struct
     */
    std::stringstream toStringStream() {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(4)
               << "( " << x << "±" << x_err <<" [m],  "
               << z << "±" << z_err <<" [rad] )";
        return stream;
    }

    /**
     * @brief Return string representation of struct
     */
    std::string toString() {
        return this->toStringStream().str();
    }

    float x;
    float z;
    float x_err;
    float z_err;
};

/**
 * @brief Class responsible for compensating robot's slips and making sure it reaches its targets
 */
class SlipCompensator {
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
     * @brief Construct a new SlipCompensator object
     * 
     * @param default_error 
     * @param min_angle_error 
     * @param loop_rate 
     */
    SlipCompensator(float default_error = 0.05f, float min_angle_error = 0.087266f, int loop_rate = 30) 
                   : node_name_(SlipCompensator::rosInit("slip_compensator")) 
                   , nh_(ros::NodeHandle())
                   , loop_rate_(ros::Rate(loop_rate))
                   , default_error_(default_error)
                   , min_angle_error_(min_angle_error)
                   , target_vector_() {
        ROS_INFO("Starting up %s node:", node_name_.c_str());

        // Create service server, that receives requests from /add_relative_target topic and passes them to
        // function addRelativeTargetServer which is a method of SlipCompensator class and belongs to "this" object 
        add_rel_target_ser_ = nh_.advertiseService("/add_relative_target", &SlipCompensator::addRelativeTargetServer, this);
        ROS_INFO(" - /add_relative_target service created");

        // Create service server, that receives requests from /clear_targets topic and passes them to
        // function clearTargetsServer which is a method of SlipCompensator class and belongs to "this" object 
        clear_targets_ser_ = nh_.advertiseService("/clear_targets", &SlipCompensator::clearTargetsServer, this);
        ROS_INFO(" - /clear_targets service created");

        // Create subscriber, that receives data from /imu_data topic with queue size equal to 1 and calls back
        // function imuDataCallback which is a method of SlipCompensator class and belongs to "this" object 
        imu_data_sub_ = nh_.subscribe("/imu_data", 1, &SlipCompensator::imuDataCallback, this);
        ROS_INFO(" - /imu_data subscriber created");

        // Create subscriber, that receives data from /motors_data topic with queue size equal to 1 and calls back
        // function motorsDataCallback which is a method of SlipCompensator class and belongs to "this" object 
        motors_data_sub_ = nh_.subscribe("/motors_data", 1, &SlipCompensator::motorsDataCallback, this);
        ROS_INFO(" - /motors_data subscriber created");

        // Create publisher that sends messages of type tank_controller::motorsAutoControl to /motors_auto_control topic with message queue size equal to 1
        auto_ctrl_pub_ = nh_.advertise<tank_controller::motorsAutoControl>("/motors_auto_control", 1);
        ROS_INFO(" - /motors_auto_control publisher created");

        ROS_INFO("Initialization finished");
    }

    // Remove copy constructor and assignement operator since they won't be needed
    SlipCompensator(const SlipCompensator &) = delete;
    SlipCompensator& operator=(const SlipCompensator &) = delete;

    /**
     * @brief Destroy the SlipCompensator object and print short message
     */
    ~SlipCompensator() {
        std::cout << "|===== Node has been successfully destroyed =====" << std::endl;
    }

    /**
     * @brief Function called automatically whenever new addRelativeTarget request is received by service server.
     * There is no need for user to call this function manually!
     * @param req reference to request message, passed automatically by ROS
     * @param res reference to response message, passed automatically by ROS
     * @return true (always)
     */
    bool addRelativeTargetServer(tank_controller::addRelativeTarget::Request &req, tank_controller::addRelativeTarget::Response &res) {

        return true;
    }

    /**
     * @brief Function called automatically whenever new clearTargets request is received by service server.
     * There is no need for user to call this function manually!
     * @param req reference to request message, passed automatically by ROS
     * @param res reference to response message, passed automatically by ROS
     * @return true (always)
     */
    bool clearTargetsServer(tank_controller::clearTargets::Request &req, tank_controller::clearTargets::Response &res) {

        return true;
    }

    /**
     * @brief Function called automatically, whenever new imu data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void imuDataCallback(const tank_controller::imuData::ConstPtr &msg) {

    }

    /**
     * @brief Function called automatically, whenever new motors data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void motorsDataCallback(const tank_controller::motorsData::ConstPtr &msg) {

    }

private:
    std::string node_name_;
    
    std::vector<target> target_vector_;
    float default_error_;
    float min_angle_error_;
    
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::ServiceServer add_rel_target_ser_;
    ros::ServiceServer clear_targets_ser_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber motors_data_sub_;
    ros::Publisher auto_ctrl_pub_;
};

int main() {
    return 0;
}