#ifndef BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>

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


    //Defining callback function to update member variables
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void ground_station_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void calculated_waypoints_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void init_callback(const std_msgs::msg::Int16::SharedPtr msg);

    // Member Variables used to store latest data
    sensor_msgs::msg::NavSatFix latest_gps_data_;

};

#endif  // BOAT_CONTROL_SYSTEM_FINAL_WAYPOINTQUEUENODE_HPP_