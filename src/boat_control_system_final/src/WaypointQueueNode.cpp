// Filename: WaypointQueueNode.cpp
// Author: Kieran Pereira
// Last Modified: 01/27/2025
// Description: Implementation file for the WaypointQueueNode class, which handles the 
//              management and processing of waypoints, subscriptions for GPS and 
//              ground station data, and controls the state of autonomous navigation.

#include "WaypointQueueNode.hpp"
#include "WaypointQueue.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

WaypointQueueNode::WaypointQueueNode() : Node("waypoint_queue_node")
{
    // Initialize the publisher
    waypoint_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/unscreened_coordinates", 
        10);

    reached_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/reached_state", 10);

    // Subscription to receive waypoints from the ground station
    ground_station_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/boat/target_coordinates", 
        10,
        std::bind(&WaypointQueueNode::ground_station_callback, this, std::placeholders::_1));

    // Subscription for Initialization data
    init_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/auto_init", 
        10,
        std::bind(&WaypointQueueNode::initCallback, this, std::placeholders::_1));

    // Subscription to check the boat's execution state
    executing_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/executing_state", 
        10,
        std::bind(&WaypointQueueNode::executing_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "WaypointQueueNode initialized successfully.");
}

void WaypointQueueNode::ground_station_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Process ground station waypoint data
    WaypointQueue::getInstance().add_waypoint(*msg); // Add waypoint to the queue
    RCLCPP_INFO(this->get_logger(), "Received waypoint from ground station: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
    std_msgs::msg::Bool reached_msg;
    reached_msg.data = false;
    reached_state_publisher_->publish(reached_msg);
    RCLCPP_INFO(this->get_logger(), "Published reached_state: false");
}

void WaypointQueueNode::initCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // Process initialization data
    if (msg->data == "False")
    {
        WaypointQueue::getInstance().close_autonomous_mode();
        RCLCPP_INFO(this->get_logger(), "Autonomous mode disabled. Waypoint queue cleared.");
    }
    else if (msg->data == "True")
    {
        RCLCPP_INFO(this->get_logger(), "Autonomous mode enabled.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid initialization message received: %s", msg->data.c_str());
    }
}

void WaypointQueueNode::executing_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) // Execution state is true
    {
        // Publish the next waypoint from the queue
        if (!WaypointQueue::getInstance().is_empty())
        {
            auto next_waypoint = WaypointQueue::getInstance().get_next_waypoint();
            waypoint_publisher_->publish(next_waypoint);

            RCLCPP_INFO(this->get_logger(), "Published waypoint: [lat: %.6f, lon: %.6f, alt: %.2f]",
                        next_waypoint.latitude, next_waypoint.longitude, next_waypoint.altitude);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Waypoint queue is empty. No waypoint to publish.");
        }
    }
    else // Execution state is false
    {
        RCLCPP_INFO(this->get_logger(), "Execution state set to false. Pausing waypoint publishing.");
        RCLCPP_INFO(this->get_logger(), "Logging current queue contents:");

        // Log the current contents of the queue
        WaypointQueue::getInstance().log_queue();
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointQueueNode>());
    rclcpp::shutdown();
    return 0;
}
