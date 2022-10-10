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
 * @brief Struct for holding PID specific variables (parameters, integrals etc.)
 * 
 */
struct PIDvariables {
public:
    /**
     * @brief Construct a new PIDvariables struct, used for holding PID specific variables
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derrivative gain
     */
    PIDvariables(const float kp, const float ki, const float kd) 
                : kp(kp)
                , ki(ki)
                , kd(kd)
                , err_integ(0.0f)
                , err_prev(0.0f) {}

    /**
     * @brief Set integral, previous error and sterring to 0
     */
    void reset() {
        err_integ = 0.0f;
        err_prev = 0.0f;
    }

    /**
     * @brief Calculate steering based on internal variables and provided target, process value and time difference
     * @param sp current target
     * @param pv current process value
     * @param time_diff time difference between this and past loop
     * @return float steering
     */
    float calculateSteering(float sp, float pv, float time_diff) {
        // Calculate current error
        float err = sp - pv;

        // Calculate integral
        err_integ = err_integ + (err * time_diff);

        // Calculate derrivative
        float err_der = (err - err_prev) / time_diff;

        // Update previous error value
        err_prev = err;

        // Calculate steering
        return kp * err + ki * err_integ + kd * err_der;
    }

public:
    float kp;
    float ki;
    float kd;

    float err_integ;
    float err_prev;
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
                   , stop_(true)
                   , last_imu_callback_(ros::Time::now())
                   , time_diff_(0.0f)
                   , target_vector_()
                   , traveled_distance_(0.0f)
                   , current_angle_(0.0f)
                   , angular_pid_(1, 0, 0)
                   , linear_pid_(1, 0, 0) {
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
        // fill in response header stamp
        res.header.stamp = ros::Time::now();
        
        // Try to add add point at the end of target vector
        try {
            target_vector_.push_back(target(req.distance, fmod(req.angle, M_PI), req.error, atan2(req.error, req.distance)));
        
        // If failed return appropriate message
        } catch (...) {
            res.feedback = "Failed to add point, target queue is full";
            ROS_WARN("Failed to add point, target queue is full");

            return true;
        } 
        
        // Else fill in response data
        res.feedback = "Added point " + target_vector_.back().toString(); 
        ROS_INFO(res.feedback.c_str()); 

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
        // Fill in response header stamp
        res.header.stamp = ros::Time::now();

        // If not requested to do any clearing, simply return current target vector size
        if(!req.clearCurrent && !req.clearQueued) {
            res.feedback = "There are currently " + std::to_string(target_vector_.size()) + " targets in queue (including current target)";
            ROS_INFO(res.feedback.c_str());

        // If requested to only clear current target remove first element from target vector
        } else if(req.clearCurrent && !req.clearQueued) {
            if(!target_vector_.empty()) {
                targetReached(true);
                res.feedback = "Current target has been removed";
                ROS_WARN("Current target has been removed");
            } else {
                res.feedback = "Target vector is empty - there was no target to remove";
            }

        // If requested to clear queued targets remove all but first element of target vector
        } else if(!req.clearCurrent && req.clearQueued) {
            if(!target_vector_.empty()) {
                int elems = target_vector_.size();
                target_vector_.erase(target_vector_.begin() + 1);
                res.feedback = "Removed " + std::to_string(elems - 1) + " queued elements";
                ROS_WARN(res.feedback.c_str());
            } else {
                res.feedback = "Queue empty - there was nothing to remove";
            }

        // If requested to clearboth current and queued targets clear vector
        } else if(req.clearCurrent && req.clearQueued) {
            if(!target_vector_.empty()) {
                int elems = target_vector_.size();
                target_vector_.clear();
                res.feedback = "Removed " + std::to_string(elems) + " elements from target vector";
                ROS_WARN(res.feedback.c_str());
            } else {
                res.feedback = "Target vector empty - there was nothing to remove";
            }
        }

        return true;
    }

    /**
     * @brief Function called automatically, whenever new imu data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void imuDataCallback(const tank_controller::imuData::ConstPtr &msg) {
        // In current version there is no filtering, only changing velocity to distance
        time_diff_ = (msg->header.stamp - last_imu_callback_).toSec();
        last_imu_callback_ = msg->header.stamp;
        traveled_distance_ = traveled_distance_ + (msg->linear.x * time_diff_);

        current_angle_ = msg->rotation.z;
    }

    /**
     * @brief Function called automatically, whenever new motors data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void motorsDataCallback(const tank_controller::motorsData::ConstPtr &msg) {
        // In current version only get global stop value
        stop_ = msg->isStopped;
    }

    /**
     * @brief Function called whenever current target has been reached to remove it from target vector
     * @param reset_pid whether to reset pid controllers or have them act continuously
     */
    void targetReached(bool reset_pid = false) {
        target_vector_.erase(target_vector_.begin());
        traveled_distance_ = 0.0f;
        last_imu_callback_ = ros::Time::now();
        time_diff_ = 0.0f;

        if(reset_pid) {
            angular_pid_.reset();
            linear_pid_.reset();
        }
    }

    /**
     * @brief Function called inside loop to calculate current steering value and publish it to /motors_auto_control topic
     */
    void calculateAndPublishSteering() {
        // Create message and fill in header data
        tank_controller::motorsAutoControl msg;
        msg.header.stamp = ros::Time::now();

        // If target vector is empty or stop is true robot needs to wait in place
        if (target_vector_.empty() || stop_) {
            // Set steering values to 0
            msg.linear.x = 0.0f;
            msg.angular.z = 0.0f;

        // Otherwise do actual calculations
        } else {
            // Calculate angular steering
            msg.angular.z = angular_pid_.calculateSteering(target_vector_[0].z, current_angle_, time_diff_);

            // If robot is near correct orientation start moving
            if(fabs(target_vector_[0].z - current_angle_) < min_angle_error_) {
                msg.linear.x = linear_pid_.calculateSteering(target_vector_[0].x, traveled_distance_, time_diff_);
            } else {
                msg.linear.x = 0.0f;
            }

            // If robot is close enough to target mark target as reached
            if(fabs(target_vector_[0].x - traveled_distance_) < target_vector_[0].x_err) {
                targetReached(false); // Do not reset pid controllers
            }
        }

        // Publish message
        auto_ctrl_pub_.publish(msg);
    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        ROS_INFO("Main program loop has been started");

        while(ros::ok()) {
            // Get new messages
            ros::spinOnce();

            // Do calculations and publish message
            calculateAndPublishSteering();

            // Sleep remaining loop time
            loop_rate_.sleep();
        }

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
    
    std::vector<target> target_vector_;
    float default_error_;
    float min_angle_error_;

    bool stop_;
    ros::Time last_imu_callback_;
    float time_diff_;

    float traveled_distance_;
    float current_angle_;

    PIDvariables angular_pid_;
    PIDvariables linear_pid_;
    
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::ServiceServer add_rel_target_ser_;
    ros::ServiceServer clear_targets_ser_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber motors_data_sub_;
    ros::Publisher auto_ctrl_pub_;
};

int main() {
    SlipCompensator slip_compensator = {};
    slip_compensator.run();
}