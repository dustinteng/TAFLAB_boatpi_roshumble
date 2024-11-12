// Filename: WaypointQueueNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Implementation file for the WaypointQueueNode class, which handles the 
//              management and processing of waypoints, subscriptions for GPS and 
//              ground station data, and controls the state of autonomous navigation.


#include "WaypointQueueNode.hpp"
#include "WaypointQueue.hpp"
#include <std_msgs/msg/string.hpp>  // Include standard message type
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

WaypointQueueNode::WaypointQueueNode() : Node("waypoint_queue_node")
{
    // Initialize the publisher
    waypoint_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/unscreened_coordinates", 
        10);

    // Subscription to receive waypoints from the ground station
    ground_station_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ground_targets", 
        10,
        std::bind(&WaypointQueueNode::ground_station_callback, this, std::placeholders::_1));

    // Subscription for Initialisation data
    init_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
        "/auto_init", 
        10,
        std::bind(&WaypointQueueNode::initCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publisher initialized.");

    // Subscription for Initialisation data
    executing_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/boat_executing_state", 
        10,
        std::bind(&WaypointQueueNode::executing_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Boat execution state subscriber (WaypointQueueNode) started.");
}

void WaypointQueueNode::ground_station_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Process ground station waypoint data
    sensor_msgs::msg::NavSatFix new_waypoint = *msg;
    
    RCLCPP_INFO(this->get_logger(), "Received waypoint from ground station.");
}

void WaypointQueueNode::initCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    // Process initialisation data
    if(msg->data != 1)
    {
        WaypointQueue::getInstance().close_autonomous_mode();
    }
    RCLCPP_INFO(this->get_logger(), "Received init message with data: %d", msg->data);
}

void WaypointQueueNode::executing_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   //This Logic dictates that if message is true, the boat is ready to recieve a new waypoint
    if(msg->data)
    {
        waypoint_publisher_->publish(WaypointQueue::getInstance().get_next_waypoint());
    }else
    {
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointQueueNode>());
    rclcpp::shutdown();
    return 0;
}
