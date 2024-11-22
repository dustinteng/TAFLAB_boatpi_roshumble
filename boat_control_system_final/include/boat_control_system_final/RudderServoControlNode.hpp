// Filename: RudderServoControlNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for RudderServoControlNode class, which handles rudder control and waypoint navigation.

#ifndef BOAT_CONTROL_SYSTEM_FINAL_RUDDERSERVOCONTROLNODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_RUDDERSERVOCONTROLNODE_HPP_

#include <thread>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>



class RudderServoControlNode : public rclcpp::Node
{
public:
    RudderServoControlNode();
    ~RudderServoControlNode();

    // Main function for handling waypoint execution
    void executeWaypoints();

    // Getter functions for current data
    std_msgs::msg::Float32 get_magnetometer_heading();
    sensor_msgs::msg::NavSatFix get_curr_pos();
    std_msgs::msg::Float32 get_wind_angle();

private:
    // Callback functions for subscribers
    void finalWaypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void windCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void initCallback(const std_msgs::msg::String::SharedPtr msg);

    // Helper function for turning the boat toward a desired heading
    void turnBoat(sensor_msgs::msg::NavSatFix target_position);

    // Helper function to set the rudder servo angle
    void setRudderServo(float angle);


    // Logging helpers
    bool shouldLog(const std::string& topic_name);
    const int log_count_interval_ = 10;
    std::unordered_map<std::string, int> message_counters_;


    // Member variables for storing the latest data
    sensor_msgs::msg::NavSatFix latest_gps_data_;
    sensor_msgs::msg::NavSatFix latest_waypoint_data_;
    std_msgs::msg::Float32 latest_magnetometer_data_;
    std_msgs::msg::Float32 latest_wind_data_;
    std::deque<sensor_msgs::msg::NavSatFix> servo_queue_;

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr executing_state_publisher_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr final_waypoint_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr magnetometer_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr init_subscriber_;

    std_msgs::msg::Bool zero_sail_state_needed;

    // Thread for waypoint execution
    std::thread execution_thread_;
    std_msgs::msg::String autonomous_mode_;

    // Mutex for thread-safe queue access
    std::mutex queue_mutex_;
};

#endif // BOAT_CONTROL_SYSTEM_FINAL_RUDDERSERVOCONTROLNODE_HPP_
