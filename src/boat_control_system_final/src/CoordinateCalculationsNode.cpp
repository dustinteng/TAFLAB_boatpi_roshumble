// Filename: CoordinateCalculationsNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: A ROS2 node implementation for performing coordinate calculations related 
//              to navigation, including path planning and data processing for GPS, wind, 
//              and magnetometer inputs.

#include <vector>
#include <chrono>
#include <thread>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "CoordinateCalculationsNode.hpp"
#include "Coordinate_Calculations.h"
#include "WaypointQueue.hpp"
#include "RudderServoControlNode.hpp"


CoordinateCalculationsNode::CoordinateCalculationsNode() : Node("coordinate_calculations_node")
{
    // Initialize the publisher
    final_waypoint_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/rudder_servo_control", 10);

    // Subscription for GPS data
    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 
        10,
        std::bind(&CoordinateCalculationsNode::gpsCallback, this, std::placeholders::_1));

    // Subscription for Wind Angle Data
    wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/wind_data", 
        10,
        std::bind(&CoordinateCalculationsNode::windCallback, this, std::placeholders::_1));

    // Subscription for Magnetometer Heading Data
    magnetometer_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/as5600_angle", 
        10,
        std::bind(&CoordinateCalculationsNode::magnetometerCallback, this, std::placeholders::_1));

    // Subscription for Unscreened Coordinates from Waypoint Queue
    waypoint_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/unscreened_coordinates", 
        10,
        std::bind(&CoordinateCalculationsNode::waypointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publisher initialized.");
}

// Main Callback function used for handling data from WaypointQueueNode
void CoordinateCalculationsNode::waypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_waypoint_data_ = *msg;
    std::vector<sensor_msgs::msg::NavSatFix> waypoints = CoordinateCalculations::getInstance().plan_path(get_curr_pos(),latest_waypoint_data_);

    while (!waypoints.empty()) 
    {
        // Access the first element of the vector
        sensor_msgs::msg::NavSatFix current_waypoint = waypoints.front();
        // Publish the current waypoint
        final_waypoint_publisher_->publish(current_waypoint);

        // Remove the first element from the vector
        waypoints.erase(waypoints.begin());
        
        // Log the waypoint being published
        RCLCPP_INFO(this->get_logger(), "Published waypoint to RudderServoControlNode: [lat: %.6f, lon: %.6f, alt: %.2f]",
                    current_waypoint.latitude, current_waypoint.longitude, current_waypoint.altitude);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// GPS callback function
void CoordinateCalculationsNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

    latest_gps_data_ = *msg;

    if (shouldLog("gps")) 
    {
        RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
    }
}

// Wind Angle callback function
void CoordinateCalculationsNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    if (shouldLog("wind")) 
    {
    RCLCPP_INFO(this->get_logger(), "Received Wind angle heading data: [angle: %.6f]",
                msg->data);
    }
}

// Magnetometer heading callback function
void CoordinateCalculationsNode::magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_magnetometer_data_ = *msg;  // Update the latest Magnetometer data
    if (shouldLog("magnetometer")) 
    {
    RCLCPP_INFO(this->get_logger(), "Received Magnetometer heading data: [heading: %.6f]",
                msg->data);
    }
}

sensor_msgs::msg::NavSatFix CoordinateCalculationsNode::get_latest_waypoint()
{
    return CoordinateCalculationsNode::latest_waypoint_data_;
}


// Defining Getter Functions for most recent data
std_msgs::msg::Float32 CoordinateCalculationsNode::get_wind_angle()
{
    return CoordinateCalculationsNode::latest_wind_data_;
}

std_msgs::msg::Float32 CoordinateCalculationsNode::get_magnetometer_heading()
{
    return CoordinateCalculationsNode::latest_magnetometer_data_;
}

sensor_msgs::msg::NavSatFix CoordinateCalculationsNode::get_curr_pos()
{
    return CoordinateCalculationsNode::latest_gps_data_;
}


bool CoordinateCalculationsNode::shouldLog(const std::string& topic_name)
{
    // Increment the message count for the topic
    message_counters_[topic_name]++;

    // Check if the message count reaches the specified log interval
    if (message_counters_[topic_name] >= log_count_interval_) {
        message_counters_[topic_name] = 0; // Reset the counter for this topic
        return true; // Log this message
    }

    return false; // Do not log this message
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}