#ifndef COORDINATE_CALCULATIONS_NODE_HPP
#define COORDINATE_CALCULATIONS_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "Datatypes.hpp"
#include "CoordinateCalculations.h"

class CoordinateCalculationsNode : public rclcpp::Node
{
public:
    // Constructor
    CoordinateCalculationsNode();

private:
    // Callbacks for subscribing to data
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // Function to get the current heading
    float get_heading_lis3mdl();

    // Function to get the current GPS coordinate
    Datatypes::Coordinate get_curr_coordinate();

    // Member variables to store the current GPS position and heading
    Datatypes::Coordinate current_position_;
    float current_heading_;

    // ROS 2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_subscription_;
};

#endif // COORDINATE_CALCULATIONS_NODE_HPP
