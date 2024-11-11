// Filename: CoordinateCalculationsNode.hpp
// Author: Kieran Pereira
// Date: 10/11/2024
// Description: Header file for the CoordinateCalculationsNode class, handling calculations and interactions with ROS 2.

#ifndef COORDINATE_CALCULATIONS_NODE_HPP
#define COORDINATE_CALCULATIONS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "DataTypes.hpp"
#include "Coordinate_Calculations.h"
#include <vector>

class CoordinateCalculationsNode : public rclcpp::Node
{
public:
    /*
    @brief Constructor for CoordinateCalculationsNode, initializes the node, subscriptions, and publisher.
    */
    CoordinateCalculationsNode();

private:
    // Callback functions for subscribing to data
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void wind_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Helper functions to access updated data
    float get_heading_lis3mdl();
    Datatypes::Coordinate get_curr_coordinate();
    float get_avg_angle();

    // Member variables to store the current GPS position and heading
    Datatypes::Coordinate current_position_;
    float current_heading_;
    float wind_angle_;  // Stores the wind angle

    // ROS 2 subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_publisher_;
};

#endif // COORDINATE_CALCULATIONS_NODE_HPP
