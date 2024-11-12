// Filename: CoordinateCalculationsNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: A ROS2 node implementation for performing coordinate calculations related 
//              to navigation, including path planning and data processing for GPS, wind, 
//              and magnetometer inputs.


#include "CoordinateCalculationsNode.hpp"
#include "WaypointQueue.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

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
        "/magnetometer_data", 
        10,
        std::bind(&CoordinateCalculationsNode::magnetometerCallback, this, std::placeholders::_1));

    // Subscription for Unscreened Coordinates from Waypoint Queue
    waypoint_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/unscreened_coordinates", 
        10,
        std::bind(&CoordinateCalculationsNode::waypointCallback, this, std::placeholders::_1));

    // Subscription for Initialisation data
    init_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
        "/auto_init", 
        10,
        std::bind(&CoordinateCalculationsNode::initCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publisher initialized.");
}

// GPS callback function
void CoordinateCalculationsNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_gps_data_ = *msg;  // Update the latest GPS data
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

// Wind Angle callback function
void CoordinateCalculationsNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    RCLCPP_INFO(this->get_logger(), "Received Wind angle heading data: [angle: %.6f]",
                msg->data);
}

// Magnetometer heading callback function
void CoordinateCalculationsNode::magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_magnetometer_data_ = *msg;  // Update the latest Magnetometer data
    RCLCPP_INFO(this->get_logger(), "Received Magnetometer heading data: [heading: %.6f]",
                msg->data);
}

// //
// void WaypointQueueNode::calculated_waypoints_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
// {
//     // Process calculated waypoint data
//     RCLCPP_INFO(this->get_logger(), "Received calculated waypoint.");
// }

void CoordinateCalculationsNode::waypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_waypoint_data_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received latest waypoint data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

void CoordinateCalculationsNode::initCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    // Process initialisation data

    RCLCPP_INFO(this->get_logger(), "Received init message with data: %d", 
                msg->data);
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}