// Filename: SailServoControlNode.hpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Header file for the SailServoControlNode class.

#ifndef BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
#define BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class SailServoControlNode : public rclcpp::Node
{
public:
    SailServoControlNode();
    void setSailServo(float angle);

private:
    void windCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // Member variables
    std_msgs::msg::Float32 latest_wind_data_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sail_angle_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wind_subscriber_;

    bool autonomous_mode_;
};

#endif // BOAT_CONTROL_SYSTEM_FINAL_SAILSERVOCONTROLNODE_HPP_
