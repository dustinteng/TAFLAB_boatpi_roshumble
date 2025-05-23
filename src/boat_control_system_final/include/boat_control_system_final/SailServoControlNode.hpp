// Filename: SailServoControlNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for the SailServoControlNode class.

#ifndef BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "taflab_msgs/msg/control_data.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <vector>
#include <string>

struct SailData {
    float windAngle;
    float optimalSailPosition;
};

class SailServoControlNode : public rclcpp::Node {
public:
    SailServoControlNode();
    void setSailServo(float angle);

private:
    void windCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void loadSailData();
    void stateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    float interpolateSailPosition(float windAngle, const SailData& lower, const SailData& upper);
    float getOptimalSailPosition(float windAngle);

    // Member variables
    std::vector<SailData> sailData;
    rclcpp::Publisher<taflab_msgs::msg::ControlData>::SharedPtr sail_angle_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_subscriber_;
    std_msgs::msg::Float32 latest_wind_data_;
    std_msgs::msg::Bool latest_state_;
    std::string filename;
    // Logging helpers
    bool shouldLog(const std::string& topic_name);
    const int log_count_interval_ = 10;
    std::unordered_map<std::string, int> message_counters_;
    

};

#endif // BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
