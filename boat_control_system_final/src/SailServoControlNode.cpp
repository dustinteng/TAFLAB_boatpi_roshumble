// Filename: SailServoControlNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Implementation of SailServoControlNode for controlling sail angle based on wind data.

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

#include "SailServoControlNode.hpp"
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

SailServoControlNode::SailServoControlNode() : Node("sail_servo_control_node")
{
    // Initialize the publisher for sail angle
    sail_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "/sail_servo_commands", 10);

    // Subscription for Wind Angle Data
    wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/wind_data", 
        10,
        std::bind(&SailServoControlNode::windCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SailServoControlNode initialized and ready.");

    SailServoControlNode::loadSailData();
}

// Wind Angle callback function
void SailServoControlNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    setSailServo(getOptimalSailPosition(latest_wind_data_.data));  // Use the `data` field
    RCLCPP_INFO(this->get_logger(), "Wind data received. Setting sail servo to angle: %.2f", msg->data);
}

// Set the sail servo angle
void SailServoControlNode::setSailServo(float angle)
{
    std_msgs::msg::Float32 msg;
    msg.data = angle;
    sail_angle_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Setting sail servo to angle: %.2f", angle);
}

void SailServoControlNode::loadSailData()
{
    std::ifstream file(filename);
    std::string line;

    if(!file.is_open())
    {
        throw std::runtime_error("Could not open file");
    }

    // Skip the header line
    std::getline(file, line);

    // Read each line of the file
    while(std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string item;
        SailData data;

        // Read wind angle
        std::getline(ss, item, ',');
        data.windAngle = std::stoi(item);

        // Read optimal sail position
        std::getline(ss, item, ',');
        data.optimalSailPosition = std::stoi(item);

        sailData.push_back(data);
    }

    file.close();
}

int SailServoControlNode::interpolateSailPosition(int windAngle, const SailData& lower, const SailData& upper)
{
    // Linear interpolation formula
    return lower.optimalSailPosition + (windAngle - lower.windAngle) * 
           (upper.optimalSailPosition - lower.optimalSailPosition) / 
           (upper.windAngle - lower.windAngle);
}

int SailServoControlNode::getOptimalSailPosition(int windAngle)
{
    // Check if exact match exists
    for(const auto& data : sailData)
    {
        if (data.windAngle == windAngle) {
            return data.optimalSailPosition;
        }
    }

    // If no exact match, find the closest two angles for interpolation
    for(size_t i = 0; i < sailData.size() - 1; ++i)
    {
        if(sailData[i].windAngle < windAngle && sailData[i + 1].windAngle > windAngle)
        {
            return interpolateSailPosition(windAngle, sailData[i], sailData[i + 1]);
        }
    }
    return -1;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailServoControlNode>());
    rclcpp::shutdown();
    return 0;
}
