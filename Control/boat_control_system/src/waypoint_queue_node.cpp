// waypoint_queue_node.cpp

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"

#include "WaypointQueue.h"

class WaypointQueueNode : public rclcpp::Node
{
public:
    WaypointQueueNode()
        : Node("waypoint_queue_node")
    {
        // Publisher for next waypoint
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/next_waypoint", 10);

        // Timer to publish waypoints at intervals
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointQueueNode::publish_waypoint, this));

        // Initialize waypoints (example waypoints)
        waypoint_queue_.add_waypoint({37.4275, -122.1697}); // Example coordinate
        waypoint_queue_.add_waypoint({37.4280, -122.1700});
        waypoint_queue_.initialize_autonomous_mode();
    }

private:
    void publish_waypoint()
    {
        if (!waypoint_queue_.is_empty())
        {
            auto next_wp = waypoint_queue_.get_next_waypoint();
            auto waypoint_msg = geometry_msgs::msg::Point();
            waypoint_msg.x = next_wp.latitude;
            waypoint_msg.y = next_wp.longitude;
            waypoint_publisher_->publish(waypoint_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No more waypoints.");
        }
    }

    WaypointQueue waypoint_queue_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointQueueNode>());
    rclcpp::shutdown();
    return 0;
}
