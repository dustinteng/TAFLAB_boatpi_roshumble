#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "Coordinate_Calculations.h"  // Include your coordinate calculations file
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>

class CoordinateCalculationsNode : public rclcpp::Node
{
public:
    CoordinateCalculationsNode() : Node("coordinate_calculations_node")
    {
        // Subscribe to GPS data
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&CoordinateCalculationsNode::gps_callback, this, std::placeholders::_1));

        // Subscribe to the next waypoint
        waypoint_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/next_waypoint", 10, std::bind(&CoordinateCalculationsNode::waypoint_callback, this, std::placeholders::_1));

        // Subscribe to wind data
        wind_subscription_ = this->create_subscription()

        // Publisher for additional waypoints
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/planned_waypoints", 10);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        current_position_.latitude = msg->latitude;
        current_position_.longitude = msg->longitude;
        RCLCPP_INFO(this->get_logger(), "Received GPS data: [%.6f, %.6f]", msg->latitude, msg->longitude);
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_heading_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received heading: %.2f", msg->data);
    }

    // Replacing the direct function calls in your logic
    float get_heading_lis3mdl()
    {
        return current_heading_;  // Use updated heading data from the subscriber
    }

    Datatypes::Coordinate get_curr_coordinate()
    {
        return current_position_;  // Use updated GPS data from the subscriber
    }

    // Class members to store received data
    Datatypes::Coordinate current_position_;
    float current_heading_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_publisher_;

    Datatypes::Coordinate current_position_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}
