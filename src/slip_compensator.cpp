#include "ros/ros.h"
#include "tank_controller/imuData.h"
#include "tank_controller/motorsData.h"
#include "tank_controller/motorsAutoControl.h"
#include "tank_controller/remainingDistance.h"
#include "tank_controller/addRelativeTarget.h"
#include "tank_controller/clearTargets.h"

#include <iostream>
#include <sstream>
#include <math.h>
#include <queue>

// 0.9 * pi
#define SEPARATE_ANGLE 2.8274333882f

/**
 * @brief Simple struct to allow target's data to be stored in std::vector
 */
struct relativeTarget {
    /**
     * @brief Construct a new relativeTarget struct with from data
     */
    relativeTarget(float x, float z, float x_err, float z_err)
          : x(x)
          , z(z)
          , x_err(x_err)
          , z_err(z_err) {}

    /**
     * @brief Construct a new relativeTarget struct from other struct
     * @param other 
     */
    relativeTarget(const relativeTarget &other)
                  : x(other.x)
                  , z(other.z)
                  , x_err(other.x_err)
                  , z_err(other.z_err) {}

    /**
     * @brief Construct a new relativeTarget struct from other struct
     * @param other 
     */
    relativeTarget& operator=(const relativeTarget &other) {
        x = other.x;
        z = other.z;
        x_err = other.x_err;
        z_err = other.z_err;
        return *this;
    }

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
                , err_prev(0.0f)
                , u(0.0f) {}

    /**
     * @brief Set integral, previous error and sterring to 0
     */
    void reset() {
        err_integ = 0.0f;
        err_prev = 0.0f;
        u = 0.0f;
    }

public:
    float kp;
    float ki;
    float kd;

    float err_integ;
    float err_prev;
    float u;
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
     * @brief Construct a new SlipCompensator object, that is object of a lass responsible for compensating robot's slips and making sure it reaches its targets
     * @param default_distance_error default value of distance error (used if service client didn't specify one)
     * @param default_angle_error_ default value of angle error (used if service client didn't specify one)
     * @param acceptable_angle_error angle error at which robot will start to move (robot won't drive forwards as long as angle error is greater than this value)
     * @param loop_rate rate at which program should run
     */
    SlipCompensator(float default_distance_error = 0.05f, float default_angle_error_ = 0.017453f, float acceptable_angle_error = 0.087266f, int loop_rate = 30) 
                   : node_name_(SlipCompensator::rosInit("slip_compensator")) 
                   , nh_(ros::NodeHandle())
                   , loop_rate_(ros::Rate(loop_rate))
                   , default_distance_error_(default_distance_error)
                   , default_angle_error_(default_angle_error_)
                   , acceptable_angle_error_(acceptable_angle_error)
                   , stop_(true)
                   , last_imu_callback_(0.0f)
                   , time_diff_(0.0f)
                   , target_queue_()
                   , traveled_distance_(0.0f)
                   , current_angle_(0.0f)
                   , angle_offset_(0.0f)
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

        // Create publisher that sends messages of type tank_controller::remainingDistance to /remaining_distance topic with message queue size equal to 1
        remain_dist_pub_= nh_.advertise<tank_controller::remainingDistance>("/remaining_distance", 1);
        ROS_INFO(" - /remaining_distance publisher created");

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
     * @brief Function called whenever a target has been reached or cleared
     * @param reset_pid reset pid controllers' internal variables
     */
    void targetReached(bool reset_pid = false) {
        // Remove current target
        if (!target_queue_.empty()) {
            target_queue_.pop();
        }

        // Set angle offset to current angle
        angle_offset_ = current_angle_;

        // Set traveled distance to 0
        traveled_distance_ = 0.0f;

        // If asked to, reset pid internat variables
        if (reset_pid) {
            linear_pid_.reset();
            angular_pid_.reset();
        }
    }

    /**
     * @brief Function called automatically whenever new addRelativeTarget request is received by service server.
     * There is no need for user to call this function manually!
     * @param req reference to request message, passed automatically by ROS
     * @param res reference to response message, passed automatically by ROS
     * @return true (always)
     */
    bool addRelativeTargetServer(tank_controller::addRelativeTarget::Request &req, tank_controller::addRelativeTarget::Response &res) {
        // Fill in response header
        res.header.stamp = ros::Time::now();
        
        // First make sure angle is within (-pi, pi) range
        float angle = fmod(req.angle, 2.0f * M_PIf32);
        angle = fabs(angle) > M_PIf32 ? std::copysignf(M_PIf32, angle) - angle : angle;

        // To avoid problems with angles close to pi add two points (one for partial rotation, other to finish rotation and start moving)
        if (fabs(angle) > SEPARATE_ANGLE) {
            // Get error values (angle error should be divided by 2 since two points will be added)
            float angle_error = req.angle_error > 0.0f ? req.angle_error * 0.5f : default_angle_error_ * 0.5f;
            float distance_error = req.distance_error > 0.0f ? req.distance_error : default_distance_error_;

            // Get angle values for both points
            float first_angle = std::copysignf(SEPARATE_ANGLE, angle);
            float second_angle = angle - first_angle;

            // Try to add points to queue
            try {
                target_queue_.push(relativeTarget(0.0f, first_angle, distance_error, angle_error));
                target_queue_.push(relativeTarget(req.distance, second_angle, distance_error, angle_error));

            // If failed return appropriate message
            } catch(...) {
                res.feedback = "Failed to add point, target queue is full";
                ROS_WARN("Failed to add point, target queue is full");

                return true;
            }

            // Otherwise inform that two pints have been added
            res.feedback = "Added two points to prevent PID controller from continous spin";
            ROS_INFO("Added two points to prevent PID controller from continous spin");

            return true;

        // If angle is not too big add only one point
        } else {
            float angle_error = req.angle_error > 0.0f ? req.angle_error : default_angle_error_;
            float distance_error = req.distance_error > 0.0f ? req.distance_error : default_distance_error_;

            // Try to add point
            try {
                target_queue_.push(relativeTarget(req.distance, angle, distance_error, angle_error));
            
            // If failed return appropriate message
            } catch(...) {
                res.feedback = "Failed to add point, target queue is full";
                ROS_WARN("Failed to add point, target queue is full");

                return true;
            }

            // Otherwise inform that point has been added
            res.feedback = "Point has been successfully added";
            ROS_INFO("Point has been successfully added");

            return true;
        }
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

        // If not requested to do any clearing, simply return current target queue size
        if(!req.clearCurrent && !req.clearQueued) {
            res.feedback = "There are currently " + std::to_string(target_queue_.size()) + " targets in queue (including current target)";
            ROS_INFO(res.feedback.c_str());

        // If requested to only clear current target remove first element from target queue
        } else if(req.clearCurrent && !req.clearQueued) {
            if(!target_queue_.empty()) {
                // The easiest way to clear current target is to asume it has been already reached
                targetReached(true);

                // Send feedback
                res.feedback = "Current target has been removed";
                ROS_WARN("Current target has been removed");
            } else {
                res.feedback = "Target queue is empty - there was no target to remove";
            }

        // If requested to clear queued targets remove all but first element of target vector
        } else if(!req.clearCurrent && req.clearQueued) {
            if(!target_queue_.empty()) {
                // Get size of current queue
                int elems = target_queue_.size();

                // Create new queue with containing only first element from the old queue
                std::queue<relativeTarget> new_target_queue;
                new_target_queue.push(target_queue_.front());

                // Swap new and old queue
                std::swap(target_queue_, new_target_queue);

                // Send feedback
                res.feedback = "Removed " + std::to_string(elems - 1) + " queued elements";
                ROS_WARN(res.feedback.c_str());
            } else {
                res.feedback = "Target queue empty - there was nothing to remove";
            }

        // If requested to clearboth current and queued targets clear vector
        } else if(req.clearCurrent && req.clearQueued) {
            if(!target_queue_.empty()) {
                // Get size of current queue
                int elems = target_queue_.size();

                // Create new empty queue and swap it with the old one
                std::queue<relativeTarget> new_target_queue;
                std::swap(target_queue_, new_target_queue);

                // Send feedback
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
        // Update time-based variables
        time_diff_ = (msg->header.stamp - last_imu_callback_).toSec();
        last_imu_callback_ = msg->header.stamp;

        // Update traveled distance and current angle (as of this verison do it without any filtering)
        traveled_distance_ = traveled_distance_ + (msg->linear.x * time_diff_);
        current_angle_ = msg->rotation.z;
    }

    /**
     * @brief Function called automatically, whenever new motors data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void motorsDataCallback(const tank_controller::motorsData::ConstPtr &msg) {
        // Enable/disable stop and (if needed) update current angle offset
        if (!stop_ && msg->isStopped) {
            stop_ = true;
        } else if (stop_ && !msg->isStopped) {
            if (target_queue_.empty()) {
                angle_offset_ = current_angle_;
            } else {
                angle_offset_ = current_angle_ + (target_queue_.front().z - angular_pid_.err_prev);
            }
            stop_ = false;
        }

        // As of this version rest of motors data is not used
    }

    /**
     * @brief Function caled insiede loop to publish current automatic steering
     */
    void motorsAutoControlPublish() {
        // Create message and fill in header
        tank_controller::motorsAutoControl msg;
        msg.header.stamp = ros::Time::now();

        // Fill in current steering and publish message
        msg.linear.x = linear_pid_.u;
        msg.angular.z = angular_pid_.u;
        auto_ctrl_pub_.publish(msg);
    }

    /**
     * @brief Function called inside loop to publish distance and angle remaining to current targetd
     */
    void remainingDistancePublish() {
        // Create message and fill in header
        tank_controller::remainingDistance msg;
        msg.header.stamp = ros::Time::now();

        // Fill in data and publish message
        msg.distance = linear_pid_.err_prev;
        msg.angle = angular_pid_.err_prev;
        remain_dist_pub_.publish(msg);
    }

    /**
     * @brief Function called inside loop to calculate current steering values
     */
    void calculateSteering() {

    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        ROS_INFO("Main program loop has been started");

        while (ros::ok()) {
            // Calculate current steering
            calculateSteering();

            // Publish steering and remaining distance
            motorsAutoControlPublish();
            remainingDistancePublish();

            // Wait for subscribers
            ros::spinOnce();

            // Sleep for the remaining loop time
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
    
    std::queue<relativeTarget> target_queue_;
    float default_distance_error_;
    float default_angle_error_;
    float acceptable_angle_error_;

    bool stop_;
    ros::Time last_imu_callback_;
    float time_diff_;

    float traveled_distance_;
    float current_angle_;
    float angle_offset_;

    PIDvariables angular_pid_;
    PIDvariables linear_pid_;
    
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    ros::ServiceServer add_rel_target_ser_;
    ros::ServiceServer clear_targets_ser_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber motors_data_sub_;
    ros::Publisher auto_ctrl_pub_;
    ros::Publisher remain_dist_pub_;
};

int main() {
    SlipCompensator slip_compensator = {};
    slip_compensator.run();
}