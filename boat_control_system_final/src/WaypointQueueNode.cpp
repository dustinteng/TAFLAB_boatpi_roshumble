#include "WaypointQueueNode.hpp"
#include <std_msgs/msg/string.hpp>  // Include standard message type
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>

WaypointQueueNode::WaypointQueueNode() : Node("waypoint_queue_node")
{
    // Initialize the publisher
    waypoint_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/servo_control", 10);

    // Subscription to receive waypoints from the ground station
    ground_station_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ground_targets", 
        10,
        std::bind(&WaypointQueueNode::ground_station_callback, this, std::placeholders::_1));

    // Subscription to receive calculated waypoints from coordinate_calculations
    calculated_waypoint_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/calculated_waypoints", 
        10,
        std::bind(&WaypointQueueNode::calculated_waypoints_callback, this, std::placeholders::_1));

    // Subscription for GPS data
    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 
        10,
        std::bind(&WaypointQueueNode::gpsCallback, this, std::placeholders::_1));

    // Subscription for Initialisation data
    init_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
        "/auto_init", 
        10,
        std::bind(&WaypointQueueNode::init_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publisher initialized.");
}

// Define the callback functions
void WaypointQueueNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_gps_data_ = *msg;  // Update the latest GPS data
    RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
}

void WaypointQueueNode::ground_station_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Process ground station waypoint data
    RCLCPP_INFO(this->get_logger(), "Received waypoint from ground station.");
}

void WaypointQueueNode::calculated_waypoints_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Process calculated waypoint data
    RCLCPP_INFO(this->get_logger(), "Received calculated waypoint.");
}

void WaypointQueueNode::init_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    // Process initialisation data
    RCLCPP_INFO(this->get_logger(), "Received init message with data: %d", msg->data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointQueueNode>());
    rclcpp::shutdown();
    return 0;
}
