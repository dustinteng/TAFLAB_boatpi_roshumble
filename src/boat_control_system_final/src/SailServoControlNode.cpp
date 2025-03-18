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
#include <std_msgs/msg/bool.hpp>
#include "taflab_msgs/msg/control_data.hpp"

SailServoControlNode::SailServoControlNode() : Node("sail_servo_control_node")
{

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("boat_control_system_final");
    // Now initialize filename with the correct path
    filename = package_share_directory + "/SailAngleData.csv";

    // Initialize the publisher for sail angle
    sail_angle_publisher_ = this->create_publisher<taflab_msgs::msg::ControlData>(
        "/boatcontrol", 10);

    // Subscription for Wind Angle Data
    wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/as5600_angle", 
        10,
        std::bind(&SailServoControlNode::windCallback, this, std::placeholders::_1));

    // Subscription to check if the boat has reached its final waypoint (and if so needes to set its sail to neutral)
    reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/reached_state", 
        10,
        std::bind(&SailServoControlNode::stateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SailServoControlNode initialized and ready.");

    SailServoControlNode::loadSailData();
}

// Wind Angle callback function
void SailServoControlNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    float sail_angle = getOptimalSailPosition(latest_wind_data_.data);
    setSailServo(sail_angle);  // Use the `data` field

    if (shouldLog("wind")) 
    {
        RCLCPP_INFO(this->get_logger(), "Wind data received. Setting sail servo to angle: %.2f", sail_angle);
        RCLCPP_INFO(this->get_logger(), "Message Data is: %.2f", msg->data);
    }
}
void SailServoControlNode::stateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    latest_state_.data = msg->data;  // Update the latest state data
    RCLCPP_INFO(this->get_logger(), "State Received. Boat has %s its final waypoint.", msg->data ? "reached" : "not reached");
}


// Set the sail servo angle
void SailServoControlNode::setSailServo(float angle)
{
    taflab_msgs::msg::ControlData control_msg;
    control_msg.servo_sail = angle;  // Set sail angle
    control_msg.servo_rudder = 0.0f; // Default rudder angle
    control_msg.esc = 0.0f;          // Default ESC value

    sail_angle_publisher_->publish(control_msg);

    RCLCPP_INFO(this->get_logger(), "Published sail servo angle: %.2f", angle);
}

void SailServoControlNode::loadSailData()
{
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
        throw std::runtime_error("Could not open file: " + filename);
    }

    // Skip the header line if present
    std::getline(file, line);

    // Read each line of the file
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string item;
        SailData data;

        // Read wind angle
        if (std::getline(ss, item, ','))
        {
            try
            {
                data.windAngle = std::stof(item);  // Convert to float
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid data for wind angle: %s", item.c_str());
                continue;  // Skip this line if conversion fails
            }
        }

        // Read optimal sail position
        if (std::getline(ss, item, ','))
        {
            try
            {
                data.optimalSailPosition = std::stof(item);  // Convert to float
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid data for optimal sail position: %s", item.c_str());
                continue;  // Skip this line if conversion fails
            }
        }

        sailData.push_back(data);  // Add the valid data to the vector
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Sail data loaded successfully.");
}



float SailServoControlNode::interpolateSailPosition(float windAngle, const SailData& lower, const SailData& upper)
{
    // Linear interpolation formula
    return lower.optimalSailPosition + (windAngle - lower.windAngle) * 
           (upper.optimalSailPosition - lower.optimalSailPosition) / 
           (upper.windAngle - lower.windAngle);
}

float SailServoControlNode::getOptimalSailPosition(float windAngle)
{
    if (latest_state_.data) // If the boat has reached its last waypoint, set the sail directly towards the wind
    {
        setSailServo(latest_wind_data_.data);
        return -1.0f;
    }
    else
    {
        // Check if exact match exists
        for (const auto& data : sailData)
        {
            if (data.windAngle == windAngle) {
                return data.optimalSailPosition;
            }
        }

        // If no exact match, find the closest two angles for interpolation
        for (size_t i = 0; i < sailData.size() - 1; ++i)
        {
            if (sailData[i].windAngle < windAngle && sailData[i + 1].windAngle > windAngle)
            {
                return interpolateSailPosition(windAngle, sailData[i], sailData[i + 1]);
            }
        }
        return -1.0f;
    }
}





bool SailServoControlNode::shouldLog(const std::string& topic_name)
{
    // Increment the message count for the topic
    message_counters_[topic_name]++;

    // Check if the message count reaches the specified log interval
    if (message_counters_[topic_name] >= log_count_interval_) {
        message_counters_[topic_name] = 0; // Reset the counter for this topic
        return true; // Log this message
    }

    return false; // Do not log this message
}





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SailServoControlNode>());
    rclcpp::shutdown();
    return 0;
}
