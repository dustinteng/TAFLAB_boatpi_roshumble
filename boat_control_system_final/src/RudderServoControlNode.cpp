// Filename: RudderServoControlNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: A ROS2 node implementation for performing coordinate calculations related 
//              to navigation, including path planning and data processing for GPS, wind, 
//              and magnetometer inputs.



#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

RudderServoControlNode::RudderServoControlNode() : Node("servo_control_node")
{
    // Initialize the publisher
    rudder_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "/rudder_servo_control",
        10);

    // Publisher for executing state
    executing_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/executing_state",
        10);

    // Subscription for GPS data
    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 
        10,
        std::bind(&RudderServoControlNode::gpsCallback, this, std::placeholders::_1));

    // Subscription for Magnetometer Heading Data
    magnetometer_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/magnetometer_data", 
        10,
        std::bind(&RudderServoControlNode::magnetometerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publisher initialized.");
}

// GPS callback function
void RudderServoControlNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_gps_data_ = *msg;  // Update the latest GPS data
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

// Wind Angle callback function
void RudderServoControlNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}