// Filename: RudderServoControlNode.cpp
// Author: Kieran Pereira
// Last Modified: 11/11/2024
// Description: Implementation of RudderServoControlNode for controlling rudder angle based on waypoints and heading.

#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include "taflab_msgs/msg/control_data.hpp"

#include "RudderServoControlNode.hpp"
#include "RudderControlHelper.hpp"


RudderServoControlNode::RudderServoControlNode() : Node("rudder_servo_control_node")
{
    // Initialize the publisher for rudder angle
    rudder_angle_publisher_ = this->create_publisher<taflab_msgs::msg::ControlData>(
        "/boatcontrol", 10);

    // Initialize the publisher for the executing state
    executing_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/executing_state", 10);

    // Initialize the publisher for the reached state
    reached_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/reached_state", 10);

    // Main Subscription for waypoints set from coordinate calculations
    final_waypoint_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/rudder_servo_control",
        10,
        std::bind(&RudderServoControlNode::finalWaypointCallback, this, std::placeholders::_1));


    // Subscription for Wind Angle Data
    wind_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/as5600_angle", 
        10,
        std::bind(&RudderServoControlNode::windCallback, this, std::placeholders::_1));

    // Subscription for GPS data
    gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        10,
        std::bind(&RudderServoControlNode::gpsCallback, this, std::placeholders::_1));

    // Subscription for Magnetometer Heading Data
    magnetometer_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/witmotion_heading",
        10,
        std::bind(&RudderServoControlNode::magnetometerCallback, this, std::placeholders::_1));

    // Subscription for Initialisation data
    init_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/auto_init", 
        10,
        std::bind(&RudderServoControlNode::initCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribers and publishers initialized.");

    // Start the execution thread for waypoint handling
    execution_thread_ = std::thread(&RudderServoControlNode::executeWaypoints, this);
    RudderServoControlNode::boat_running = true;
}

RudderServoControlNode::~RudderServoControlNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down RudderServoControlNode...");

    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        autonomous_mode = "False";
        boat_running = false;
    }

    // Ensure execution thread has time to check shutdown flag
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "RudderServoControlNode shutdown complete.");
}








// Main Callback function used to handle incoming processed waypoints from coordinate calculations
void RudderServoControlNode::finalWaypointCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    servo_queue_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Received waypoint: [lat: %.6f, lon: %.6f, alt: %.2f]",
                msg->latitude, msg->longitude, msg->altitude);
    RCLCPP_INFO(this->get_logger(), "Waypoint queue size: %zu", servo_queue_.size());
}

// GPS callback function
void RudderServoControlNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latest_gps_data_ = *msg;
    if (shouldLog("gps")) 
    {
        RCLCPP_INFO(this->get_logger(), "Received GPS data: [lat: %.6f, lon: %.6f, alt: %.2f]",
                    msg->latitude, msg->longitude, msg->altitude);
    }
    RCLCPP_INFO(this->get_logger(), "Updated GPS data successfully.");
}

// Magnetometer heading callback function
void RudderServoControlNode::magnetometerCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_magnetometer_data_ = *msg;
    if (shouldLog("magnetometer")) 
    {
        RCLCPP_INFO(this->get_logger(), "Received Magnetometer heading data: [heading: %.6f]",
                    msg->data);
    }
    RCLCPP_INFO(this->get_logger(), "Updated Magnetometer data successfully.");
}

// Wind Angle callback function
void RudderServoControlNode::windCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_wind_data_ = *msg;  // Update the latest wind angle data
    if (shouldLog("wind")) 
    {
        RCLCPP_INFO(this->get_logger(), "Received Wind angle heading data: [angle: %.6f]",
                    msg->data);
    }
    RCLCPP_INFO(this->get_logger(), "Updated Wind data successfully.");
}

void RudderServoControlNode::initCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // Process initialization data
    if (msg->data == "False")
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);  // Ensure thread-safe access
        servo_queue_.clear();
        RudderServoControlNode::autonomous_mode = "False";
        RCLCPP_INFO(this->get_logger(), "Servo queue cleared upon initialization.");
    }
    else{
        RudderServoControlNode::autonomous_mode = "True";
    }
    RCLCPP_INFO(this->get_logger(), "Received init message with data: %s", msg->data.c_str());
}





//Helper Functions

// Function to turn the boat toward the desired heading
void RudderServoControlNode::turnBoat(sensor_msgs::msg::NavSatFix target_position)
{
    float turn_angle;
    std::string turn_direction;
    bool turn_complete = false;

    while (!turn_complete && rclcpp::ok() && boat_running) 
    {
        if (!rclcpp::ok() || !boat_running) {
            RCLCPP_INFO(this->get_logger(), "ROS2 Shutdown detected, stopping turnBoat.");
            break;
        }

        // Create local copies to prevent data corruption
        float local_heading = get_magnetometer_heading().data;
        RCLCPP_INFO(this->get_logger(), "Turn Angle: %.2f", local_heading);

        sensor_msgs::msg::NavSatFix local_position = get_curr_pos();

        std::tie(turn_direction, turn_angle) = RudderControlHelper::getInstance().calculate_directional_bearing(target_position, local_heading, local_position);

        float desired_heading = RudderControlHelper::getInstance().calculate_bearing(local_position, target_position);

        RCLCPP_INFO(this->get_logger(), "Turn Angle: %.2f, Direction: %s, Desired: %.2f", turn_angle, turn_direction.c_str(), desired_heading);

        if (abs(turn_angle) <= 1.0f) 
        {
            setRudderServo(0);
            turn_complete = true;
            break;
        }

        int rudder_angle = 0;

        if (turn_angle > 10.0f)
        {
            rudder_angle = 45;
        } else if (turn_angle > 5.0f)
        {
            rudder_angle = 30;
        } else if (turn_angle > 2.0f)
        {
            rudder_angle = 15;
        }

        // Apply direction
        if (turn_direction == "left")
        {
            rudder_angle = -rudder_angle;
        }

        setRudderServo(rudder_angle);


        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    setRudderServo(0);
    RCLCPP_INFO(this->get_logger(), "Turn complete.");
}




// Main function for handling waypoint execution
// Main function for handling waypoint execution
void RudderServoControlNode::executeWaypoints()
{
    while (rclcpp::ok() && boat_running)
    {
        if (!rclcpp::ok() || !boat_running) {
            RCLCPP_INFO(this->get_logger(), "ROS2 Shutdown detected, stopping waypoint execution.");
            break;
        }


        
        if(RudderServoControlNode::autonomous_mode == "True")
        {
            
            RCLCPP_INFO(this->get_logger(), "Starting waypoint execution loop.");

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (servo_queue_.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "No waypoints in queue, checking proximity to last waypoint.");
                    sensor_msgs::msg::NavSatFix curr_position = get_curr_pos();
                    if (RudderControlHelper::getInstance().calculate_distance(curr_position, latest_waypoint_data_) <= 0.05)
                    {
                        RCLCPP_INFO(this->get_logger(), "Near last waypoint, setting rudder to idle.");
                        setRudderServo(get_wind_angle().data);
                        zero_sail_state_needed.data = true;
                        executing_state_publisher_->publish(zero_sail_state_needed);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep before next check
                    continue; // Keep looping to check for new waypoints
                }
            }

            if (zero_sail_state_needed.data)
            {
                RCLCPP_INFO(this->get_logger(), "Resetting sail state.");
                zero_sail_state_needed.data = false;
                executing_state_publisher_->publish(zero_sail_state_needed);
            }

            while (rclcpp::ok() && boat_running)
            {
                if (!rclcpp::ok() || !boat_running) { // EDITED: Stop processing waypoints on shutdown
                    RCLCPP_INFO(this->get_logger(), "ROS2 Shutdown detected, stopping waypoint processing.");
                    break;
                }
                
                sensor_msgs::msg::NavSatFix next_waypoint;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    if (servo_queue_.empty()) {
                        break;
                    }
                    next_waypoint = servo_queue_.front();
                    servo_queue_.pop_front();
                }

                RCLCPP_INFO(this->get_logger(), "Processing next waypoint: [lat: %.6f, lon: %.6f, alt: %.2f]", 
                            next_waypoint.latitude, next_waypoint.longitude, next_waypoint.altitude);

                // Use turnBoat to navigate towards the next waypoint
                turnBoat(next_waypoint);

                // Wait until the boat reaches the waypoint
                while (RudderControlHelper::getInstance().calculate_distance(get_curr_pos(), next_waypoint) > 0.05 && rclcpp::ok() && boat_running) 
                {
                    if (!rclcpp::ok() || !boat_running) { // EDITED: Break if shutdown is detected
                        RCLCPP_INFO(this->get_logger(), "ROS2 Shutdown detected inside waypoint navigation, stopping.");
                        break;
                    }
                    RCLCPP_INFO(this->get_logger(), "Navigating to waypoint, current distance: %.2f", 
                                RudderControlHelper::getInstance().calculate_distance(get_curr_pos(), next_waypoint));
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                // Ensure boat stops execution immediately if shutdown is triggered
                if (!rclcpp::ok() || !boat_running) { // EDITED: Exit if ROS2 is shutting down
                    break;
                }

                // Set the rudder to neutral once the waypoint is reached
                setRudderServo(0);
                RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next.");

                if (servo_queue_.empty()) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;  // Set the value of the Bool message
                    executing_state_publisher_->publish(msg);  // Publish the message
                    RCLCPP_INFO(this->get_logger(), "Waypoint queue empty, publishing execution state false.");

                    std_msgs::msg::Bool reached_msg;
                    reached_msg.data = true;
                    reached_state_publisher_->publish(reached_msg);
                    RCLCPP_INFO(this->get_logger(), "Waypoint queue empty, publishing reached_state true.");
                }
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Boat Turning off");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}







// Set the rudder servo angle
void RudderServoControlNode::setRudderServo(float angle)
{
    taflab_msgs::msg::ControlData control_msg;
    control_msg.servo_sail = 0.0f;  // Set sail angle
    control_msg.servo_rudder = angle; // Default rudder angle
    control_msg.esc = 0.0f;          // Default ESC value
    rudder_angle_publisher_->publish(control_msg);

    RCLCPP_INFO(this->get_logger(), "Setting rudder servo to angle: %.2f", angle);
}

// Getter functions

std_msgs::msg::Float32 RudderServoControlNode::get_magnetometer_heading()
{
    return RudderServoControlNode::latest_magnetometer_data_;
}

sensor_msgs::msg::NavSatFix RudderServoControlNode::get_curr_pos()
{
    return RudderServoControlNode::latest_gps_data_;
}

// Defining Getter Functions for most recent data
std_msgs::msg::Float32 RudderServoControlNode::get_wind_angle()
{
    return RudderServoControlNode::latest_wind_data_;
}






bool RudderServoControlNode::shouldLog(const std::string& topic_name)
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
    auto node = std::make_shared<RudderServoControlNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down node...");

    node.reset();  // ✅ Ensure destructor is called
    rclcpp::shutdown();

    return 0;
}
