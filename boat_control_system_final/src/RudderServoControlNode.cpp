// Filename: RudderServoControlNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Implementation of RudderServoControlNode for controlling rudder angle based on waypoints and heading.

#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "RudderServoControlNode.hpp"
#include "RudderControlHelper.hpp"

RudderServoControlNode::RudderServoControlNode() : Node("rudder_servo_control_node"), autonomous_mode_(true)
{
    // Initialize the publisher for rudder angle
    rudder_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "/rudder_servo_commands", 10);




    // Main Subscription for waypoints set from coordinate calculations
    final_waypoint_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/rudder_servo_control",
        10,
        std::bind(&RudderServoControlNode::finalWaypointCallback, this, std::placeholders::_1));

    // Initialize the subscriber for the executing state
    executing_state_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/executing_state", 10,
        std::bind(&RudderServoControlNode::executingStateCallback, this, std::placeholders::_1));

    // Subscription for Wind Angle Data
    wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/wind_data", 
        10,
        std::bind(&RudderServoControlNode::windCallback, this, std::placeholders::_1));

    // Subscription for GPS data
    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        10,
        std::bind(&RudderServoControlNode::gpsCallback, this, std::placeholders::_1));

    // Subscription for Magnetometer Heading Data
    magnetometer_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/as5600_angle",
        10,
        std::bind(&RudderServoControlNode::magnetometerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publishers initialized.");

    // Start the execution thread for waypoint handling
    execution_thread_ = std::thread(&RudderServoControlNode::executeWaypoints, this);
}

RudderServoControlNode::~RudderServoControlNode()
{
    
    if (execution_thread_.joinable())
    {
        execution_thread_.join();
    }
}






// Main Callback function used to handle incoming processed waypoints from coordinate calculations
void RudderServoControlNode::finalWaypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    servo_queue_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Received waypoint: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

// GPS callback function
void RudderServoControlNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_gps_data_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

// Magnetometer heading callback function
void RudderServoControlNode::magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_magnetometer_data_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received Magnetometer heading data: [heading: %.6f]",
                msg->data);
}

// Wind Angle callback function
void RudderServoControlNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    RCLCPP_INFO(this->get_logger(), "Received Wind angle heading data: [angle: %.6f]",
                msg->data);
}

void RudderServoControlNode::executingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Implementation for handling the executing state
    RCLCPP_INFO(this->get_logger(), "Received executing state: %s", msg->data ? "True" : "False");
}






//Helper Functions

// Function to turn the boat toward the desired heading
void RudderServoControlNode::turnBoat(sensor_msgs::msg::NavSatFix target_position)
{
    float turn_angle;
    std::string turn_direction;

    while (true) {
        // Calculate the turn direction and angle to the target position
        std::tie(turn_direction, turn_angle) = RudderControlHelper::getInstance().calculate_directional_bearing(target_position);

        float desired_angle_heading = RudderControlHelper::getInstance().calculate_bearing(get_curr_pos(),target_position);
        // Break out of the loop if the turn angle is within an acceptable range
        if (abs(turn_angle) <= 1.0f) {
            setRudderServo(0);  // Neutral rudder when aligned
            break;
        }

        // Adjust the rudder based on the turn direction and angle
        if (turn_angle > 10.0f) 
        {
            setRudderServo(turn_direction == "right" ? 45 : -45);
        } else if (turn_angle > 5.0f) 
        {
            setRudderServo(turn_direction == "right" ? 30 : -30);
        } else if (turn_angle > 2.0f) 
        {
            setRudderServo(turn_direction == "right" ? 15 : -15);
        } else {
            setRudderServo(0);
        }

        // Delay for smoother adjustments
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Safety check for significant drift
        float current_heading = get_magnetometer_heading().data;
        if (abs(current_heading - desired_angle_heading) > 10.0f) {
            RCLCPP_WARN(this->get_logger(), "Correcting for significant drift: Current heading %.2f, Desired heading %.2f", current_heading, desired_angle_heading);
        }
    }

    // Ensure the rudder is set to neutral after turning is complete
    setRudderServo(0);
}


// Main function for handling waypoint execution
void RudderServoControlNode::executeWaypoints()
{
    while (autonomous_mode_)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (servo_queue_.empty())
        {
            sensor_msgs::msg::NavSatFix curr_position = get_curr_pos();
            if (RudderControlHelper::getInstance().calculate_distance(curr_position, latest_waypoint_data_) <= 0.05)
            {
                setRudderServo(get_wind_angle().data);  // Set rudder to idle if near the destination
                break;
            }
            continue;
        }

        while (!servo_queue_.empty())
        {
            sensor_msgs::msg::NavSatFix next_waypoint = servo_queue_.front();
            servo_queue_.pop_front();

            // Use turnBoat to navigate towards the next waypoint
            turnBoat(next_waypoint);

            // Wait until the boat reaches the waypoint
            while (RudderControlHelper::getInstance().calculate_distance(get_curr_pos(), next_waypoint) > 0.05)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Set the rudder to neutral once the waypoint is reached
            setRudderServo(0);
            RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next.");
        }
    }
}









// Set the rudder servo angle
void RudderServoControlNode::setRudderServo(float angle)
{
    std_msgs::msg::Float32 msg;
    msg.data = angle;
    rudder_angle_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Setting rudder servo to angle: %.2f", angle);
}

// Getter functions

std_msgs::msg::Float32 RudderServoControlNode::get_magnetometer_heading()
{
    return RudderServoControlNode::latest_magnetometer_data_;
}

sensor_msgs::msg::NavSatFix RudderServoControlNode::get_curr_pos()
{
    return RudderServoControlNode::latest_gps_data_;
}

// Defining Getter Functions for most recent data
std_msgs::msg::Float32 RudderServoControlNode::get_wind_angle()
{
    return RudderServoControlNode::latest_wind_data_;
}






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RudderServoControlNode>());
    rclcpp::shutdown();
    return 0;
}