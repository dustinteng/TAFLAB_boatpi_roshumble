// Filename: SailServoControlNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for the SailServoControlNode class.

#ifndef BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>

struct SailData {
    int windAngle;
    int optimalSailPosition;
};

class SailServoControlNode : public rclcpp::Node {
public:
    SailServoControlNode();
    void setSailServo(float angle);

private:
    void windCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void loadSailData();
    void stateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    int interpolateSailPosition(int windAngle, const SailData& lower, const SailData& upper);
    int getOptimalSailPosition(int windAngle);

    // Member variables
    std::vector<SailData> sailData;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sail_angle_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_subscriber_;
    std_msgs::msg::Float32 latest_wind_data_;
    std_msgs::msg::Bool latest_state_;
    std::string filename = "boat_control_system_final/SailAngleData.csv"; 
};

#endif // BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
