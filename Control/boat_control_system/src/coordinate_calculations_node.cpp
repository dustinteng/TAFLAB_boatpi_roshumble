// coordinate_calculations_node.cpp

#include <rclcpp/rclcpp.hpp>


#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"

#include "Coordinate_Calculations.hpp"

class CoordinateCalculationsNode : public rclcpp::Node
{
public:
    CoordinateCalculationsNode()
        : Node("coordinate_calculations_node")
    {
        // Subscribe to GPS data
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&CoordinateCalculationsNode::gps_callback, this, std::placeholders::_1));

        // Subscribe to next waypoint
        waypoint_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/next_waypoint", 10, std::bind(&CoordinateCalculationsNode::waypoint_callback, this, std::placeholders::_1));

        // Publisher for bearing
        bearing_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/bearing_to_waypoint", 10);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        current_position_.latitude = msg->latitude;
        current_position_.longitude = msg->longitude;
    }

    void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        Datatypes::Coordinate target_waypoint;
        target_waypoint.latitude = msg->x;
        target_waypoint.longitude = msg->y;

        float bearing = CoordinateCalculations::calculate_bearing(current_position_, target_waypoint);

        auto bearing_msg = std_msgs::msg::Float32();
        bearing_msg.data = bearing;
        bearing_publisher_->publish(bearing_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bearing_publisher_;

    Datatypes::Coordinate current_position_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateCalculationsNode>());
    rclcpp::shutdown();
    return 0;
}
