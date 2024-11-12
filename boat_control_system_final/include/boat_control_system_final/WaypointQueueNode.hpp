// Filename: WaypointQueueNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for the WaypointQueueNode class, which implements a ROS2 node 
//              for managing waypoints, receiving GPS and calculated coordinates, and 
//              handling initialization and execution states for autonomous navigation.


#ifndef BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>


#include <deque>

class WaypointQueueNode : public rclcpp::Node
{
public:
    WaypointQueueNode();
private:

    // Listing Publishers:

    // Declare a publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr waypoint_publisher_;


    //Listing Subscribers:

    // Creating Subscription to recieve target waypoints from the ground station
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ground_station_subscriber_;

    // Creating Subscriber to recieve calculated waypoints from coordinate_calculations
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr calculated_waypoint_subscriber_;
    
    // Creating Subscriber for GPS Messages
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

    // Creating Subscriber for autonomous mode init
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr init_subscriber_;

    // Creating Subscriber to see if boat is ready to execute a new waypoint
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr executing_subscriber_;


    //Defining callback function to update member variables
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void ground_station_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    // void calculated_waypoints_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void initCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void executing_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Member Variables used to store latest data
    sensor_msgs::msg::NavSatFix latest_gps_data_;
};

#endif  // BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_