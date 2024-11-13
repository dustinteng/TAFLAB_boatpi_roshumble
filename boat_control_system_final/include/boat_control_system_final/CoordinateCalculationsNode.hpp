// Filename: CoordinateCalculationsNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for the CoordinateCalculationsNode class, which implements 
//              functions for coordinate calculations, path planning, and handling data 
//              from GPS, wind angle, and magnetometer sensors for navigation purposes.


#ifndef BOAT_CONTROL_SYSTEM_FINAL_COORDINATECALCULATIONSNODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_COORDINATECALCULATIONSNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "RudderServoControlNode.hpp"

#include <deque>

class CoordinateCalculationsNode : public rclcpp::Node
{
public:
    CoordinateCalculationsNode();

    // Defining Getter Functions for most recent data
    std_msgs::msg::Float32 get_wind_angle();
    std_msgs::msg::Float32 get_magnetometer_heading();
    sensor_msgs::msg::NavSatFix get_curr_pos();
    sensor_msgs::msg::NavSatFix get_latest_waypoint();

   

private:

    // Listing Publishers:

    // Declare a publisher
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr final_waypoint_publisher_;


    //Listing Subscribers:

    // Creating Subscription to recieve target waypoints from the waypoint queue node
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr waypoint_subscriber_;

    // Creating Subscription to recieve wind angle data
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscriber_;

    // Creating Subscriber for magnetometer heading data
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr magnetometer_subscriber_;

    // Creating Subscriber to recieve calculated waypoints from coordinate_calculations
    // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr calculated_waypoint_subscriber_;
    
    // Creating Subscriber for GPS Messages
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;

    // Creating Subscriber for autonomous mode init
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr init_subscriber_;

    // // Creating Subscriber to see if boat is ready to execute a new waypoint
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr executing_subscriber_;


    //Main Function used to process waypoints recieved from Waypoint queue
    void waypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    //Defining callback function to update member variables
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    // void calculated_waypoints_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void initCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void windCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // Member Variables used to store latest data
    sensor_msgs::msg::NavSatFix latest_gps_data_;
    std_msgs::msg::Float32 latest_magnetometer_data_;
    std_msgs::msg::Float32 latest_wind_data_;
    sensor_msgs::msg::NavSatFix latest_waypoint_data_;


};

#endif  // BOAT_CONTROL_SYSTEM_FINAL_COORDINATECALCULATIONSNODE_HPP_