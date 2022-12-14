#include "ros/ros.h"
#include "tank_controller/imuData.h"
#include "tank_controller/motorsData.h"
#include "tank_controller/motorsAutoControl.h"
#include "tank_controller/remainingDistance.h"
#include "tank_controller/addRelativeTarget.h"
#include "tank_controller/clearTargets.h"

#include <iostream>
#include <sstream>
#include <queue>
#include <math.h>
#include <chrono>

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f

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
     * @param max_integ max integral error value
     * @param max_steering max steering value
     */
    PIDvariables(const float kp, const float ki, const float kd, const float max_integ, const float max_steering) 
                : kp(kp)
                , ki(ki)
                , kd(kd)
                , max_err_integ(max_integ)
                , max_u(max_steering)
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

    /**
     * @brief Calculate steering based on internal variables and provided target, process value and time difference
     * @param sp current error
     * @param time_diff time difference between this and past loop
     */
    void calculateSteering(float err, float time_diff) {
        // Calculate and constrain integral
        err_integ = err_integ + (err * time_diff);
        err_integ = (err_integ <= -max_err_integ ? -max_err_integ : (err_integ >= max_err_integ ? max_err_integ : err_integ));

        // Calculate derrivative
        float err_der = (err - err_prev) / time_diff;

        // Update previous error value
        err_prev = err;

        // Calculate and constrain steering
        float steering = kp * err + ki * err_integ + kd * err_der;
        u = (steering <= -max_u ? -max_u : (steering >= max_u ? max_u : steering));
    }

public:
    float kp;
    float ki;
    float kd;

    float max_err_integ;
    float max_u;

    float err_integ;
    float err_prev;
    float u;
};


/**
 * @brief Class responsible for assisting with driving by automatically reaching given targets
 */
class DriverAssist {
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
     * @brief Construct a new DriverAssist object, that is object of a class responsible for assisting with driving by automatically reaching given targets
     * @param default_distance_error default value of distance error (used if service client didn't specify one)
     * @param default_angle_error_ default value of angle error (used if service client didn't specify one)
     * @param acceptable_angle_error angle error at which robot will start to move (robot won't drive forwards as long as angle error is greater than this value)
     * @param loop_rate rate at which program should run
     */
    DriverAssist(float max_linear = 1.0f, float max_angular = 1.0f*PI, float default_distance_error = 0.1f, float default_angle_error_ = 0.0872665f, float acceptable_angle_error = 0.1745329f, int loop_rate = 30) 
                   : node_name_(DriverAssist::rosInit("driver_assist")) 
                   , nh_(ros::NodeHandle())
                   , loop_rate_(ros::Rate(loop_rate))
                   , default_distance_error_(default_distance_error)
                   , default_angle_error_(default_angle_error_)
                   , moving_angle_error_(acceptable_angle_error)
                   , stop_(true)
                   , time_diff_(0.0f)
                   , target_queue_()
                   , traveled_distance_(0.0f)
                   , relative_angle_(0.0f)
                   , prev_rotation_(0.0f)
                   , angular_pid_(1.0f, 0.1f, 0.0f, fabs(max_angular), fabs(0.75f * max_angular))
                   , linear_pid_(0.5f, 0.175f, 0.1f, fabs(max_linear), fabs(0.75f * max_linear)) {
        ROS_INFO("Starting up %s node:", node_name_.c_str());
        last_motors_callback_ = std::chrono::high_resolution_clock::now();

        // Create service server, that receives requests from /add_relative_target topic and passes them to
        // function addRelativeTargetServer which is a method of DriverAssist class and belongs to "this" object 
        add_rel_target_ser_ = nh_.advertiseService("/add_relative_target", &DriverAssist::addRelativeTargetServer, this);
        ROS_INFO(" - /add_relative_target service created");

        // Create service server, that receives requests from /clear_targets topic and passes them to
        // function clearTargetsServer which is a method of DriverAssist class and belongs to "this" object 
        clear_targets_ser_ = nh_.advertiseService("/clear_targets", &DriverAssist::clearTargetsServer, this);
        ROS_INFO(" - /clear_targets service created");

        // Create subscriber, that receives data from /imu_data topic with queue size equal to 1 and calls back
        // function imuDataCallback which is a method of DriverAssist class and belongs to "this" object 
        imu_data_sub_ = nh_.subscribe("/imu_data", 1, &DriverAssist::imuDataCallback, this);
        ROS_INFO(" - /imu_data subscriber created");

        // Create subscriber, that receives data from /motors_data topic with queue size equal to 1 and calls back
        // function motorsDataCallback which is a method of DriverAssist class and belongs to "this" object 
        motors_data_sub_ = nh_.subscribe("/motors_data", 1, &DriverAssist::motorsDataCallback, this);
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
    DriverAssist(const DriverAssist &) = delete;
    DriverAssist& operator=(const DriverAssist &) = delete;

    /**#define ONE_AND_HALF_PI 4.71238898038
     * @brief Destroy the DriverAssist object and print short message
     */
    ~DriverAssist() {
        std::cout << "|===== Node has been successfully destroyed =max_a====" << std::endl;
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
        relative_angle_ = 0.0f;

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

        // If target queue is empty, reset angle offset and traveled distance
        if (target_queue_.empty()) {
            relative_angle_ = 0.0f;
            traveled_distance_ = 0.0f;
        }
        
        // Make sure angle is within (-pi, pi) range
        float angle = fmod(req.angle, TWO_PI);
        angle = fabs(angle) > PI ? std::copysignf(PI, angle) - angle : angle;

        // Fill angle and distance error (if not specified)
        float angle_error = req.angle_error > default_angle_error_ ? req.angle_error : default_angle_error_;
        float distance_error = req.distance_error > default_distance_error_ ? req.distance_error : default_distance_error_;

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

                // Make sure robots doesn't start going backwards
                targetReached(true);

                // Send feedback
                res.feedback = "Removed " + std::to_string(elems) + " elements from target queue";
                ROS_WARN(res.feedback.c_str());
            } else {
                res.feedback = "Target queue empty - there was nothing to remove";
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
        // If not stopped update current angle (no need for filtering, sensor has built-in filter)
        if (!stop_) {
            // Calculate rotation difference
            float diff = msg->rotation.z - prev_rotation_;

            // If difference is too big it means -pi/pi breakpoint has been passed
            // In that case add or subtract 2*pi from diff value
            if (diff > PI) {
                diff = diff - TWO_PI;
            } else if (diff < -PI) {
                diff = diff + TWO_PI;
            } // Above statements could've been done in one line, but this way is more readable and much cleaner  

            // Add diff to current angle
            relative_angle_ += diff;
        }
        
        // Always update previous rotation
        prev_rotation_ = msg->rotation.z;
    }

    /**
     * @brief Function called automatically, whenever new motors data is received.
     * There is no need for user to call this function manually!
     * @param msg Passed automatically by ROS subscriber
     */
    void motorsDataCallback(const tank_controller::motorsData::ConstPtr &msg) {
        // @todo -> change to chrono timer
        // Enable/disable stop
        stop_ = msg->isStopped;

        // Update time
        std::chrono::high_resolution_clock::time_point current_callback = std::chrono::high_resolution_clock::now();
        time_diff_ = std::chrono::duration<float>(current_callback - last_motors_callback_).count();
        last_motors_callback_ = current_callback;
        
        // If not stopped update traveled distance
        if (!stop_) {
            traveled_distance_ += msg->linear.x * time_diff_;
        }
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
     * @brief Function called inside calculateSteering function to publish distance remaining to target
     */
    void remainingDistancePublish(float linear_err, float angular_err) {
        // Create message and fill in header
        tank_controller::remainingDistance msg;
        msg.header.stamp = ros::Time::now();

        // Fill in data and publish message
        msg.distance = linear_err;
        msg.angle = angular_err;
        
        remain_dist_pub_.publish(msg);
    }

    /**
     * @brief Function called inside loop to calculate current steering values
     */
    void doControl() {
        // If has target caluculate position error
        if (!target_queue_.empty()) {
            float angular_error = target_queue_.front().z - relative_angle_;
            float linear_error = target_queue_.front().x - traveled_distance_;

            // If not stopped calculate steering, otherwise set steering to 0 and send current position errors
            if (!stop_) {
                // Pre-calculate some boolean values to avoid multiple operations on floats (and for the code to be more clear)
                bool angle_ok = fabs(angular_error) < target_queue_.front().z_err;
                bool angle_acceptable = fabs(angular_error) < moving_angle_error_;
                bool rotating = angular_pid_.u != 0.0f;
                bool distance_ok = fabs(linear_error) < target_queue_.front().x_err;
                bool driving = linear_pid_.u != 0.0f;

                // If both angle and distance are OK that means target was reached
                if (angle_ok && distance_ok) {
                    targetReached();
                
                // If robot was not driving, rotate until angle is ok    
                } else if (!driving) {
                    // Calculate angular steering
                    angular_pid_.calculateSteering(angular_error, time_diff_);

                    if (!angle_ok) {
                        linear_pid_.reset();
                    } else if (rotating) {
                        angular_pid_.reset();
                        linear_pid_.reset();
                    } else {
                        linear_pid_.calculateSteering(linear_error, time_diff_);
                    }

                // If robot was driving make sure angle error is acceptable
                } else {
                    // Calculate angular steering
                    angular_pid_.calculateSteering(angular_error, time_diff_);

                    if (angle_acceptable) {
                        linear_pid_.calculateSteering(linear_error, time_diff_);
                    } else {
                        linear_pid_.reset();
                    }
                }

            } else {
                angular_pid_.reset();
                linear_pid_.reset();    
            }

            // Publish actual errors
            remainingDistancePublish(linear_error, angular_error);

        // Otherwise set steering to 0 and send NaN for distance errors
        } else {
            angular_pid_.reset();
            linear_pid_.reset();

            remainingDistancePublish(std::nanf(""), std::nanf(""));
        }
    }

    /**
     * @brief Main program loop. Call it to run nodes and entire program logic.
     * @return EXIT_SUCCESS if closed properly
     */
    int run() {
        ROS_INFO("Main program loop has been started");

        while (ros::ok()) {
            // Calculate current steering
            doControl();

            // Publish steering and remaining distance
            motorsAutoControlPublish();

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
    float moving_angle_error_;

    bool stop_;
    std::chrono::high_resolution_clock::time_point last_motors_callback_;
    float time_diff_;

    float traveled_distance_;
    float relative_angle_;
    float prev_rotation_;

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
    DriverAssist driver_assist = {};
    driver_assist.run();
}