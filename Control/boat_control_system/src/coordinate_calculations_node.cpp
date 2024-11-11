#include "coordinate_calculations_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>

CoordinateCalculationsNode::CoordinateCalculationsNode() : Node("coordinate_calculations_node")
{
    // Subscribe to GPS data
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&CoordinateCalculationsNode::gps_callback, this, std::placeholders::_1));

    // Subscribe to the next waypoint
    waypoint_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/next_waypoint", 10, std::bind(&CoordinateCalculationsNode::heading_callback, this, std::placeholders::_1));

    // Subscribe to wind data
    wind_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "/wind_data", 10, std::bind(&CoordinateCalculationsNode::wind_callback, this, std::placeholders::_1));

    // Publisher for additional waypoints
    waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/planned_waypoints", 10);
}

void CoordinateCalculationsNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    current_position_.latitude = msg->latitude;
    current_position_.longitude = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [%.6f, %.6f]", msg->latitude, msg->longitude);
}

void CoordinateCalculationsNode::heading_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_heading_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received heading: %.2f", msg->data);
}

void CoordinateCalculationsNode::wind_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    wind_angle_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received wind data: %.2f", msg->data);
}

// Helper functions to access updated data
float CoordinateCalculationsNode::get_heading_lis3mdl()
{
    return current_heading_;  // Use updated heading data from the subscriber
}

Datatypes::Coordinate CoordinateCalculationsNode::get_curr_coordinate()
{
    return current_position_;  // Use updated GPS data from the subscriber
}

float CoordinateCalculationsNode::get_avg_angle()
{
    return wind_angle_;  // Use updated wind angle data from the subscriber
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}
